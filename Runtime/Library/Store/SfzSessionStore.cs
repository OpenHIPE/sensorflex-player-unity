// SfzSessionStore.cs — session store for SFZ 2.x sessions, from a .sfz archive (Sfz mode)
// or an unpacked session directory (FileIo mode). Format: Docs/SensorFlexFormat.md.
//
// Single source of truth for SFZ-mode session lifecycle and data access.
// CameraSubsystem drives it through StartSession / Tick / StopSessionAsync;
// OcclusionSubsystem, ControlBridge, and other consumers read state through
// the proxy properties — never touching IO types directly.
//
// Private nested types (all invisible outside this file):
//   IBackendState                 — playback state contract (slot members serve live mode)
//   VideoV2BackendState           — rgb.mp4 VideoPlayer + depth.bin block decoding
//   ISessionBackend               — three-phase backend contract (Open / TryGetSessionJson / StartLoading)
//   SessionLoadState              — session lifecycle enum (Idle / Waiting / LoadingAttachments / Loading / Ready)
//   SfzTrackInfo                  — per-track metadata (name, sample interval, record count)
//   SfzAttachmentInfo             — per-attachment metadata (name, file, format)
//   SfzSessionData                — parsed session.json aggregate
//   SfzSessionJson / DTOs         — session.json and frames.jsonl deserialization types
//   ReadZipEntry                  — drains a ZipArchiveEntry into byte[]
//   SfzBackendBase                — shared loading: frames.jsonl parse, rgb_frame alignment, playback state
//   SfzFileBackend                — SFZ (ZIP archive) source; extracts large files per session
//   FileIoBackend                 — FileIo (directory) source; uses files in place
//   ScannedMeshLoaderImpl         — polls for scene_mesh attachment, drives PLY parse
//   ScannedSceneMeshLoadOperation — wraps Task-based PLY parse; TryComplete() polled each frame
//   ScannedSceneMeshData          — parsed vertex/normal/color/triangle arrays
//   PlyMeshReader                 — ASCII and binary-little-endian PLY decoder

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Video;

namespace SensorFlex.Player.Library
{
    internal static class SfzSessionStore
    {
        // ── Private state ─────────────────────────────────────────────────────
        static SessionLoadState s_LoadState = SessionLoadState.Idle;
        static ISessionBackend  s_Backend;
        static SfzSessionData   s_SessionData;
        static int              s_BufSize;
        static int              s_FramesToWait;
        static readonly ScannedMeshLoaderImpl s_MeshLoader = new();

        // ── Per-frame data (written by CameraSubsystem each frame) ────────────
        internal static long       LatestTimestampNs       = 0;
        internal static Vector4    LatestIntrinsics        = new(935.3f, 935.3f, 960f, 720f);
        internal static Vector2Int LatestTextureDimensions = new(1920, 1440);

        // ── Session state ─────────────────────────────────────────────────────
        internal static bool IsActive             => s_LoadState != SessionLoadState.Idle;
        internal static bool IsLoadingAttachments => s_LoadState == SessionLoadState.LoadingAttachments;
        internal static bool IsReady              => s_LoadState == SessionLoadState.Ready;

        // ── Ring buffer proxies (read-only for all consumers) ─────────────────
        internal static double    FrameInterval                  => s_Backend?.State?.FrameInterval ?? (1.0 / 30);
        internal static Matrix4x4 CoordConvMatrix                => s_Backend?.State?.CoordConvMatrix ?? Matrix4x4.identity;
        internal static bool      UseNegativeZForwardOpticalAxis => s_Backend?.State?.UseNegativeZForwardOpticalAxis ?? false;
        internal static int       TotalFrames                    => s_Backend?.State?.TotalFrames ?? 0;
        internal static int       BufSize                        => s_Backend?.State?.BufSize ?? 0;
        internal static Texture2D[]  Frames        => s_Backend?.State?.Frames;
        internal static byte[][]     DepthBins     => s_Backend?.State?.DepthBins;
        internal static Matrix4x4[]  Poses         => s_Backend?.State?.Poses;
        internal static Vector4[]    Intrinsics    => s_Backend?.State?.Intrinsics;
        internal static bool[]       SlotReady     => s_Backend?.State?.SlotReady;
        internal static int[]        SlotGlobalIdx => s_Backend?.State?.SlotGlobalIdx;
        internal static int          LatestGlobalIndex  => s_Backend?.State?.LatestGlobalIndex ?? -1;
        internal static int          PendingDecodeCount => s_Backend?.State?.PendingDecodeCount ?? 0;

        // ── Video mode proxies ────────────────────────────────────────────────
        internal static bool          IsVideoMode            => s_Backend?.State?.IsVideoMode ?? false;
        internal static RenderTexture VideoRgbTexture        => s_Backend?.State?.VideoRgbTexture;
        internal static void          SeekVideoFrame(int idx)=> s_Backend?.State?.SeekVideoFrame(idx);
        internal static long          VideoPlayerCurrentFrame=> s_Backend?.State?.VideoPlayerCurrentFrame ?? 0;
        internal static void PlayVideo()                         => s_Backend?.State?.PlayVideo();
        internal static void PauseVideo()                        => s_Backend?.State?.PauseVideo();
        internal static void SetVideoPlaybackSpeed(float speed)  => s_Backend?.State?.SetVideoPlaybackSpeed(speed);
        internal static void SetVideoLooping(bool loop)          => s_Backend?.State?.SetVideoLooping(loop);
        internal static void SetVideoFrameReadyCallback(Action<int> cb) => s_Backend?.State?.SetFrameReadyCallback(cb);

        // ── v2.0 depth proxies ────────────────────────────────────────────────
        internal static Texture2D V2DepthTexture                          => s_Backend?.State?.V2DepthTexture;
        internal static void      UpdateVideoDepthForFrame(int frameIdx)  => s_Backend?.State?.UpdateDepthForFrame(frameIdx);

        internal static int PlayHead
        {
            get => s_Backend?.State?.PlayHead ?? -1;
            set { if (s_Backend?.State != null) s_Backend.State.PlayHead = value; }
        }

        // ── Lifecycle (called by CameraSubsystem) ─────────────────────────────

        internal static void StartSession(ARSensorFlexSession session, int maxFramesToLoad, int framesToWait)
        {
            s_BufSize      = maxFramesToLoad;
            s_FramesToWait = framesToWait;
            s_Backend      = CreateBackend(session.SourceMode);
            s_MeshLoader.Reset();

            if (!s_Backend.Open(session))
            {
                Debug.LogError("[SF] SfzSessionStore: backend failed to open.");
                s_Backend = null;
                return;
            }

            s_LoadState = SessionLoadState.Waiting;
            Debug.Log($"[SF] SfzSessionStore: waiting for session data. mode={session.SourceMode}");
        }

        internal static void Tick()
        {
            s_Backend?.Dispatch();

            switch (s_LoadState)
            {
                case SessionLoadState.Waiting:
                    if (!s_Backend.TryGetSessionJson(out var json))
                        break;
                    if (TryParseSession(json, out s_SessionData))
                    {
                        s_LoadState = SessionLoadState.LoadingAttachments;
                        Debug.Log($"[SF] SfzSessionStore: session parsed. id={s_SessionData.SessionId} " +
                                  $"tracks={s_SessionData.Tracks.Count} attachments={s_SessionData.Attachments.Count}");
                    }
                    else
                    {
                        // A session.json that cannot be used will not become usable by retrying.
                        s_LoadState = SessionLoadState.Idle;
                        Debug.LogError("[SF] SfzSessionStore: session stopped — session.json could not be loaded.");
                    }
                    break;

                case SessionLoadState.LoadingAttachments:
                    s_MeshLoader.Tick();
                    if (s_MeshLoader.IsComplete)
                    {
                        s_Backend.StartLoading(s_SessionData, s_BufSize, s_FramesToWait);
                        s_LoadState = SessionLoadState.Loading;
                        Debug.Log("[SF] SfzSessionStore: attachments ready — starting frame streaming.");
                    }
                    break;

                case SessionLoadState.Loading:
                case SessionLoadState.Ready:
                    s_Backend.DrainMainThreadWork();
                    if (s_LoadState == SessionLoadState.Loading && s_Backend.State?.IsReady == true)
                    {
                        s_LoadState = SessionLoadState.Ready;
                        Debug.Log("[SF] SfzSessionStore: ready.");
                    }
                    break;
            }
        }

        internal static async Task StopSessionAsync()
        {
            s_LoadState   = SessionLoadState.Idle;
            s_SessionData = null;
            s_MeshLoader.Reset();
            ResetFrameData();

            var backend = s_Backend;
            s_Backend = null;

            if (backend != null)
            {
                await backend.StopAsync();
                backend.State?.DestroyTextures();
            }
        }

        internal static byte[] TryConsumeAttachment(string name) => s_Backend?.TryGetAttachmentBytes(name);

        // Resets all state without stopping any active backend.
        // Use only when no session is running (e.g., on initial Start()).
        internal static void Clear()
        {
            s_LoadState   = SessionLoadState.Idle;
            s_SessionData = null;
            s_Backend     = null;
            s_MeshLoader.Reset();
            ResetFrameData();
        }

        // ── Helpers ───────────────────────────────────────────────────────────

        static void ResetFrameData()
        {
            LatestTimestampNs       = 0;
            LatestIntrinsics        = new Vector4(935.3f, 935.3f, 960f, 720f);
            LatestTextureDimensions = new Vector2Int(1920, 1440);
        }

        static ISessionBackend CreateBackend(ARSensorFlexSession.FrameSourceMode mode) => mode switch
        {
            ARSensorFlexSession.FrameSourceMode.Sfz    => new SfzFileBackend(),
            ARSensorFlexSession.FrameSourceMode.FileIo => new FileIoBackend(),
            _ => throw new ArgumentOutOfRangeException(nameof(mode), mode, null)
        };

        // Parses session.json (SFZ 2.x — see Docs/SensorFlexFormat.md).
        static bool TryParseSession(string json, out SfzSessionData data)
        {
            data = null;
            if (string.IsNullOrEmpty(json)) return false;

            var raw = JsonUtility.FromJson<SfzSessionJson>(json);
            if (raw == null) return false;

            if (string.IsNullOrEmpty(raw.sfz_version) || !raw.sfz_version.StartsWith("2.", StringComparison.Ordinal))
            {
                Debug.LogError($"[SF] SFZ: unsupported archive version '{raw.sfz_version ?? raw.version ?? "(none)"}'. " +
                               "Only SFZ 2.x is supported; re-export older archives with the current tools.");
                return false;
            }
            if (string.IsNullOrEmpty(raw.session_id)) return false;

            double sampleInterval = raw.actual_fps > 0 ? 1.0 / raw.actual_fps : 1.0 / 60;
            var tracks = new Dictionary<string, SfzTrackInfo>
            {
                ["frames"] = new SfzTrackInfo("frames", sampleInterval, raw.frame_count)
            };

            var attachments = new Dictionary<string, SfzAttachmentInfo>();
            if (raw.attachments?.scene_mesh != null && !string.IsNullOrEmpty(raw.attachments.scene_mesh.file))
                attachments["scene_mesh"] = new SfzAttachmentInfo(
                    "scene_mesh", raw.attachments.scene_mesh.file, raw.attachments.scene_mesh.format);

            // Depth: block encodings are decoded per frame from depth.bin; video encodings
            // (depth.mp4) cannot be decoded exactly through VideoPlayer and are not played.
            string depthEncoding = raw.depth?.encoding;
            string depthBinFile  = null;
            switch (depthEncoding)
            {
                case null:
                    break;
                case "lz4_float32":
                case "lz4_uint16":
                case "zstd_float32":
                    depthBinFile = raw.depth.file ?? "depth.bin";
                    break;
                default:
                    Debug.LogWarning($"[SF] SFZ: depth encoding '{depthEncoding}' is not supported by the player — " +
                                     "replaying without depth. Record with the LZ4 depth codec to get depth.");
                    break;
            }

            // Factor that converts a decoded depth value to metres.
            float depthUnitScale = raw.depth?.units switch
            {
                "millimeters" => 0.001f,
                "centimeters" => 0.01f,
                _             => 1f,
            };

            data = new SfzSessionData(
                raw.session_id,
                tracks,
                attachments,
                rgbMp4File:     raw.rgb?.file ?? "rgb.mp4",
                depthBinFile:   depthBinFile,
                depthEncoding:  depthEncoding,
                depthUnitScale: depthUnitScale,
                rgbWidth:       raw.rgb?.width    ?? 1920,
                rgbHeight:      raw.rgb?.height   ?? 1440,
                depthWidth:     raw.depth?.width  ?? 256,
                depthHeight:    raw.depth?.height ?? 192,
                frameCount:     raw.frame_count);

            return true;
        }

        // ── Nested: Playback state ────────────────────────────────────────────

        // Playback state contract read by the camera and occlusion subsystems. The slot /
        // ring-buffer members serve frame-streaming sources (live mode, currently disabled);
        // SFZ sessions are video-driven and expose every frame as a ready slot.
        interface IBackendState
        {
            double    FrameInterval                  { get; set; }
            Matrix4x4 CoordConvMatrix                { get; set; }
            bool      UseNegativeZForwardOpticalAxis  { get; set; }
            int       TotalFrames                    { get; set; }
            int       BufSize                        { get; }
            bool      IsReady                        { get; set; }
            Texture2D[]  Frames      { get; set; }
            byte[][]     DepthBins   { get; set; }
            Matrix4x4[]  Poses       { get; set; }
            Vector4[]    Intrinsics  { get; set; }
            bool[]       SlotReady   { get; set; }
            int[]        SlotGlobalIdx { get; set; }
            int  PlayHead           { get; set; }
            int  LatestGlobalIndex  { get; set; }
            int  PendingDecodeCount { get; set; }
            void AllocateRingBuffer();
            void MarkBuffered(int framesToWait);
            void DestroyTextures();
            bool          IsVideoMode        { get; }
            RenderTexture VideoRgbTexture    { get; }
            void          SeekVideoFrame(int frameIndex);
            long          VideoPlayerCurrentFrame     { get; }
            void          PlayVideo();
            void          PauseVideo();
            void          SetVideoPlaybackSpeed(float speed);
            void          SetVideoLooping(bool loop);
            void          SetFrameReadyCallback(Action<int> cb);
            void          UpdateDepthForFrame(int frameIdx);
            Texture2D     V2DepthTexture { get; }
        }

        // ── Nested: SFZ 2.x playback state ───────────────────────────────────
        // RGB from rgb.mp4 through a VideoPlayer; depth decoded on demand from depth.bin
        // (LZ4 / Zstd blocks) into an RFloat texture in metres.

        sealed class VideoV2BackendState : IBackendState
        {
            int m_PlayHead = -1;
            int m_LatestGlobalIndex = -1;

            public double    FrameInterval                  { get; set; }
            public Matrix4x4 CoordConvMatrix                { get; set; } = Matrix4x4.identity;
            public bool      UseNegativeZForwardOpticalAxis  { get; set; }
            public int       TotalFrames                    { get; set; }
            public int       BufSize                        => TotalFrames;
            public bool      IsReady                        { get; set; }

            public Texture2D[]  Frames        { get => null; set { } }
            public byte[][]     DepthBins     { get => null; set { } }
            public Matrix4x4[]  Poses         { get; set; }
            public Vector4[]    Intrinsics    { get; set; }
            public bool[]       SlotReady     { get; set; }
            public int[]        SlotGlobalIdx { get; set; }
            public int          PendingDecodeCount { get; set; }

            public int PlayHead
            {
                get => Volatile.Read(ref m_PlayHead);
                set => Volatile.Write(ref m_PlayHead, value);
            }

            public int LatestGlobalIndex
            {
                get => Volatile.Read(ref m_LatestGlobalIndex);
                set => Volatile.Write(ref m_LatestGlobalIndex, value);
            }

            // ── Video ─────────────────────────────────────────────────────────
            public bool IsVideoMode => true;

            RenderTexture m_RgbRT;
            GameObject    m_RgbGo;
            VideoPlayer   m_RgbPlayer;
            long          m_CurrentFrame = -1;
            Action<int>   m_FrameReadyCallback;
            string        m_TempDir;   // per-session extraction dir, deleted on destroy; null for FileIo

            // ── Depth blocks (LZ4 or Zstd) ────────────────────────────────────
            SfzV2FrameRecord[] m_V2Records;
            FileStream         m_DepthStream;
            Texture2D          m_DepthTexture;
            byte[]             m_CompressedBuf;
            byte[]             m_DecodedBuf;     // one decoded frame, rows top-first as stored
            byte[]             m_UploadBuf;      // float32 metres, rows bottom-first (Unity texture order)
            int                m_DepthW, m_DepthH;
            string             m_DepthEncoding;  // "lz4_float32", "lz4_uint16" or "zstd_float32"
            float              m_DepthUnitScale = 1f;

#if UNITY_IOS && !UNITY_EDITOR
            [System.Runtime.InteropServices.DllImport("__Internal")]
            static extern UIntPtr SFDecodeBlock(IntPtr src, UIntPtr srcLen,
                                                IntPtr dst, UIntPtr dstLen, int algo);
#endif

            public RenderTexture VideoRgbTexture => m_RgbRT;
            public Texture2D     V2DepthTexture  => m_DepthTexture;

            public void AllocateRingBuffer()
            {
                SlotReady     = new bool[TotalFrames];
                SlotGlobalIdx = new int[TotalFrames];
                Poses         = new Matrix4x4[TotalFrames];
                Intrinsics    = new Vector4[TotalFrames];
                for (int i = 0; i < TotalFrames; i++)
                {
                    SlotReady[i]     = true;
                    SlotGlobalIdx[i] = i;
                    Poses[i]         = Matrix4x4.identity;
                }
            }

            public void MarkBuffered(int framesToWait) { }

            // depthBinPath may be null (no depth). tempDir, when set, is deleted with the state.
            public void InitializePlayers(
                string rgbPath, string depthBinPath, string tempDir,
                SfzV2FrameRecord[] v2Records,
                int rgbW, int rgbH, int depthW, int depthH,
                string depthEncoding, float depthUnitScale)
            {
                m_TempDir        = tempDir;
                m_V2Records      = v2Records;
                m_DepthEncoding  = depthEncoding;
                m_DepthUnitScale = depthUnitScale > 0f ? depthUnitScale : 1f;

                m_RgbRT = new RenderTexture(rgbW, rgbH, 0, RenderTextureFormat.ARGB32) { name = "SF-V2-RgbVideo" };
                m_RgbRT.Create();

                if (!string.IsNullOrEmpty(depthBinPath))
                {
                    if (File.Exists(depthBinPath))
                    {
                        int bytesPerPixel = m_DepthEncoding == "lz4_uint16" ? 2 : 4;
                        m_DepthW       = depthW;
                        m_DepthH       = depthH;
                        m_DepthStream  = File.OpenRead(depthBinPath);
                        m_DepthTexture = new Texture2D(depthW, depthH, TextureFormat.RFloat, false) { name = "SF-V2-Depth" };
                        m_DecodedBuf   = new byte[depthW * depthH * bytesPerPixel];
                        m_UploadBuf    = new byte[depthW * depthH * 4];
                    }
                    else
                    {
                        Debug.LogWarning($"[SF] V2: depth file not found at {depthBinPath} — replaying without depth.");
                    }
                }

                m_RgbGo = new GameObject("SF-VideoPlayer-V2-RGB") { hideFlags = HideFlags.HideAndDontSave };
                UnityEngine.Object.DontDestroyOnLoad(m_RgbGo);

                m_RgbPlayer = m_RgbGo.AddComponent<VideoPlayer>();
                m_RgbPlayer.source          = VideoSource.Url;
                m_RgbPlayer.url             = "file://" + rgbPath;
                m_RgbPlayer.renderMode      = VideoRenderMode.RenderTexture;
                m_RgbPlayer.targetTexture   = m_RgbRT;
                m_RgbPlayer.playOnAwake     = false;
                m_RgbPlayer.isLooping       = false;
                m_RgbPlayer.audioOutputMode = VideoAudioOutputMode.None;
                m_RgbPlayer.skipOnDrop      = false;
                m_RgbPlayer.sendFrameReadyEvents = true;
                m_RgbPlayer.frameReady          += OnRgbFrameReady;
                m_RgbPlayer.prepareCompleted    += OnRgbPrepared;
                m_RgbPlayer.Prepare();
            }

            void OnRgbFrameReady(VideoPlayer _, long frameIdx)
            {
                Volatile.Write(ref m_CurrentFrame, frameIdx);
                m_FrameReadyCallback?.Invoke((int)frameIdx);
            }

            void OnRgbPrepared(VideoPlayer _)
            {
                // Stop auto-play that some platforms trigger after Prepare despite playOnAwake=false.
                if (m_RgbPlayer.isPlaying) m_RgbPlayer.Pause();
                IsReady = true;
                Debug.Log("[SF] VideoV2BackendState: prepared. depth=" + (m_DepthTexture != null));
            }

            public void UpdateDepthForFrame(int frameIdx)
            {
                if (m_DepthStream == null || m_V2Records == null ||
                    frameIdx < 0 || frameIdx >= m_V2Records.Length) return;

                ref var rec = ref m_V2Records[frameIdx];
                if (rec.DepthByteSize <= 0) return;

                if (m_CompressedBuf == null || m_CompressedBuf.Length < rec.DepthByteSize)
                    m_CompressedBuf = new byte[rec.DepthByteSize];

                try
                {
                    m_DepthStream.Seek(rec.DepthByteOffset, SeekOrigin.Begin);
                    int totalRead = 0;
                    while (totalRead < rec.DepthByteSize)
                    {
                        int n = m_DepthStream.Read(m_CompressedBuf, totalRead, rec.DepthByteSize - totalRead);
                        if (n <= 0) break;
                        totalRead += n;
                    }
                    if (m_DepthEncoding == "zstd_float32")
                        DecodeZstdBlock(m_CompressedBuf, rec.DepthByteSize, m_DecodedBuf);
                    else
                        Lz4BlockDecoder.Decode(m_CompressedBuf, 0, rec.DepthByteSize,
                                               m_DecodedBuf, 0, m_DecodedBuf.Length);

                    if (m_DepthEncoding == "lz4_uint16")
                        ToTextureOrder(System.Runtime.InteropServices.MemoryMarshal.Cast<byte, ushort>(m_DecodedBuf));
                    else
                        ToTextureOrder(System.Runtime.InteropServices.MemoryMarshal.Cast<byte, float>(m_DecodedBuf));
                    m_DepthTexture.LoadRawTextureData(m_UploadBuf);
                    m_DepthTexture.Apply(false);
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[SF] V2 depth decode failed frame={frameIdx}: {e.Message}");
                }
            }

            // Decoded depth (rows top-first, in `units`) → m_UploadBuf as float32 metres with rows
            // bottom-first, the order LoadRawTextureData expects. Without the row flip, depth is
            // upside down relative to the color video.
            void ToTextureOrder(ReadOnlySpan<ushort> src)
            {
                var dst = System.Runtime.InteropServices.MemoryMarshal.Cast<byte, float>(m_UploadBuf);
                for (int row = 0; row < m_DepthH; row++)
                {
                    int s = row * m_DepthW, d = (m_DepthH - 1 - row) * m_DepthW;
                    for (int col = 0; col < m_DepthW; col++)
                        dst[d + col] = src[s + col] * m_DepthUnitScale;
                }
            }

            void ToTextureOrder(ReadOnlySpan<float> src)
            {
                var dst = System.Runtime.InteropServices.MemoryMarshal.Cast<byte, float>(m_UploadBuf);
                for (int row = 0; row < m_DepthH; row++)
                {
                    int s = row * m_DepthW, d = (m_DepthH - 1 - row) * m_DepthW;
                    for (int col = 0; col < m_DepthW; col++)
                        dst[d + col] = src[s + col] * m_DepthUnitScale;
                }
            }

            static void DecodeZstdBlock(byte[] src, int srcLen, byte[] dst)
            {
#if UNITY_IOS && !UNITY_EDITOR
                // GCHandle pins the arrays so the GC won't move them during the P/Invoke call.
                var hSrc = System.Runtime.InteropServices.GCHandle.Alloc(src, System.Runtime.InteropServices.GCHandleType.Pinned);
                var hDst = System.Runtime.InteropServices.GCHandle.Alloc(dst, System.Runtime.InteropServices.GCHandleType.Pinned);
                try
                {
                    SFDecodeBlock(hSrc.AddrOfPinnedObject(), (UIntPtr)srcLen,
                                  hDst.AddrOfPinnedObject(), (UIntPtr)dst.Length, 1);
                }
                finally { hSrc.Free(); hDst.Free(); }
#else
                Debug.LogWarning("[SF] Zstd depth decode not supported on this platform.");
#endif
            }

            public void SetFrameReadyCallback(Action<int> cb) { m_FrameReadyCallback = cb; }

            public void PlayVideo()  => m_RgbPlayer?.Play();

            public void PauseVideo()
            {
                if (m_RgbPlayer?.isPlaying == true) m_RgbPlayer.Pause();
            }

            public void SetVideoPlaybackSpeed(float speed)
            {
                if (m_RgbPlayer != null) m_RgbPlayer.playbackSpeed = speed;
            }

            public void SetVideoLooping(bool loop)
            {
                if (m_RgbPlayer != null) m_RgbPlayer.isLooping = loop;
            }

            // Current frame — updated via frameReady event, not the unreliable .frame getter.
            public long VideoPlayerCurrentFrame => Volatile.Read(ref m_CurrentFrame);

            // Frame seek is only used for step-forward while paused; never during normal play.
            public void SeekVideoFrame(int frameIndex)
            {
                if (m_RgbPlayer != null && m_RgbPlayer.isPrepared)
                    m_RgbPlayer.frame = (long)frameIndex;
                Volatile.Write(ref m_CurrentFrame, (long)frameIndex);
            }

            public void DestroyTextures()
            {
                if (m_RgbPlayer != null)
                {
                    m_RgbPlayer.frameReady       -= OnRgbFrameReady;
                    m_RgbPlayer.prepareCompleted -= OnRgbPrepared;
                    m_RgbPlayer.Stop();
                    m_RgbPlayer = null;
                }
                if (m_RgbGo != null) { UnityEngine.Object.Destroy(m_RgbGo); m_RgbGo = null; }
                if (m_RgbRT != null) { m_RgbRT.Release(); UnityEngine.Object.Destroy(m_RgbRT); m_RgbRT = null; }
                if (m_DepthTexture != null) { UnityEngine.Object.Destroy(m_DepthTexture); m_DepthTexture = null; }
                m_DepthStream?.Close();
                m_DepthStream = null;

                // Only files this session extracted are deleted — never a FileIo session's originals.
                if (!string.IsNullOrEmpty(m_TempDir))
                {
                    try { if (Directory.Exists(m_TempDir)) Directory.Delete(m_TempDir, recursive: true); }
                    catch (Exception e) { Debug.LogWarning($"[SF] Failed to delete temp dir {m_TempDir}: {e.Message}"); }
                    m_TempDir = null;
                }
            }
        }

        // ── Nested: SFZ 2.x frame record ─────────────────────────────────────

        struct SfzV2FrameRecord
        {
            public int        FrameIndex;
            public long       TimestampNs;
            public Vector3    Position;
            public Quaternion Rotation;
            public Vector4    Intrinsics;    // (fx, fy, cx, cy)
            public long       DepthByteOffset;
            public int        DepthByteSize;
            public int        RgbFrame;      // index into rgb.mp4; -1 = no color; RgbFrameAbsent = field missing
        }

        const int RgbFrameAbsent = int.MinValue;

        // ── Nested: Session contracts ─────────────────────────────────────────

        enum SessionLoadState { Idle, Waiting, LoadingAttachments, Loading, Ready }

        interface ISessionBackend
        {
            /// <summary>Open / validate the data source. Returns false on hard failure.</summary>
            bool Open(ARSensorFlexSession session);

            /// <summary>
            /// Returns session.json text when available; false if still waiting.
            /// Called each Tick() until it returns true.
            /// </summary>
            bool TryGetSessionJson(out string json);

            /// <summary>Prepare playback. Called once after parse.</summary>
            void StartLoading(SfzSessionData data, int bufSize, int framesToWait);

            /// <summary>Returns raw bytes for a named attachment, or null when unavailable.</summary>
            byte[] TryGetAttachmentBytes(string attachmentName);

            /// <summary>Playback state; non-null after StartLoading.</summary>
            IBackendState State { get; }

            void DrainMainThreadWork();
            void Dispatch();
            Task StopAsync();
        }

        // ── Nested: Session data model ────────────────────────────────────────

        sealed class SfzTrackInfo
        {
            public string Name           { get; }
            public double SampleInterval { get; }
            public int    RecordCount    { get; }

            internal SfzTrackInfo(string name, double sampleInterval, int recordCount)
            { Name = name; SampleInterval = sampleInterval; RecordCount = recordCount; }
        }

        sealed class SfzAttachmentInfo
        {
            public string Name   { get; }
            public string File   { get; }
            public string Format { get; }

            internal SfzAttachmentInfo(string name, string file, string format)
            { Name = name; File = file; Format = format; }
        }

        sealed class SfzSessionData
        {
            public string SessionId { get; }
            public IReadOnlyDictionary<string, SfzTrackInfo>       Tracks      { get; }
            public IReadOnlyDictionary<string, SfzAttachmentInfo>  Attachments { get; }
            internal string RgbMp4File    { get; }
            internal string DepthBinFile  { get; }  // null when there is no playable depth
            internal string DepthEncoding { get; }  // "lz4_float32", "lz4_uint16", "zstd_float32"
            internal float  DepthUnitScale { get; } // decoded value × scale = metres (lz4_uint16 mm → 0.001)
            internal int    RgbWidth      { get; }
            internal int    RgbHeight     { get; }
            internal int    DepthWidth    { get; }
            internal int    DepthHeight   { get; }
            internal int    FrameCount    { get; }

            internal SfzSessionData(
                string sessionId,
                IReadOnlyDictionary<string, SfzTrackInfo>      tracks,
                IReadOnlyDictionary<string, SfzAttachmentInfo> attachments,
                string rgbMp4File      = "rgb.mp4",
                string depthBinFile    = null,
                string depthEncoding   = null,
                float  depthUnitScale  = 1f,
                int    rgbWidth        = 1920,
                int    rgbHeight       = 1440,
                int    depthWidth      = 256,
                int    depthHeight     = 192,
                int    frameCount      = 0)
            {
                SessionId      = sessionId   ?? "session";
                Tracks         = tracks      ?? new Dictionary<string, SfzTrackInfo>();
                Attachments    = attachments ?? new Dictionary<string, SfzAttachmentInfo>();
                RgbMp4File     = rgbMp4File;
                DepthBinFile   = depthBinFile;
                DepthEncoding  = depthEncoding;
                DepthUnitScale = depthUnitScale > 0f ? depthUnitScale : 1f;
                RgbWidth       = rgbWidth;
                RgbHeight      = rgbHeight;
                DepthWidth     = depthWidth;
                DepthHeight    = depthHeight;
                FrameCount     = frameCount;
            }
        }

        // ── Nested: session.json / frames.jsonl DTOs (SFZ 2.x) ────────────────

        [Serializable] class SfzDeviceJson { public string model; public string os; public string ar_framework; }
        [Serializable] class SfzRgbJson    { public int width; public int height; public string encoding; public string file; }
        [Serializable] class SfzDepthJson  { public int width; public int height; public string encoding; public string file; public string units; public string sensor; public float depth_max_meters; }
        [Serializable] class SfzAttachmentJson  { public string file; public string format; }
        [Serializable] class SfzAttachmentsJson { public SfzAttachmentJson scene_mesh; }

        [Serializable] class SfzSessionJson
        {
            public string             sfz_version;
            public string             version;       // set only by retired 1.x archives; used for the error message
            public string             session_id;
            public string             start_time_utc;
            public SfzDeviceJson      device;
            public int                actual_fps;
            public int                frame_count;
            public SfzRgbJson         rgb;
            public SfzDepthJson       depth;
            public SfzAttachmentsJson attachments;
        }

        [Serializable] class SfzFrameLineJson
        {
            public int     frame;
            public long    ts;
            public float[] pos;
            public float[] rot;
            public float   fx, fy, cx, cy;
            public long    depth_off;
            public int     depth_sz;
            public int     rgb_frame = RgbFrameAbsent;   // absent in archives written before rgb_frame existed
        }

        static byte[] ReadZipEntry(ZipArchiveEntry entry)
        {
            var buf = new byte[(int)entry.Length];
            using var s = entry.Open();
            int total = 0;
            while (total < buf.Length)
            {
                int n = s.Read(buf, total, buf.Length - total);
                if (n <= 0) break;
                total += n;
            }
            return buf;
        }

        // ── Nested: Session backends ──────────────────────────────────────────
        // A backend only knows where session files live (inside a .sfz, or in a directory);
        // parsing frames.jsonl and building the playback state is shared.

        abstract class SfzBackendBase : ISessionBackend
        {
            protected ARSensorFlexSession m_Session;
            IBackendState m_State;

            public IBackendState State => m_State;
            public void Dispatch()            { }
            public void DrainMainThreadWork() { }
            public Task StopAsync()           => Task.CompletedTask;

            public abstract bool Open(ARSensorFlexSession session);
            public abstract bool TryGetSessionJson(out string json);

            protected abstract string BackendLabel { get; }

            /// <summary>Reads a small session file (relative to session/) into memory, or null.</summary>
            protected abstract byte[] ReadSessionFile(string relativePath);

            /// <summary>
            /// Makes a large session file available as a local path for streaming (VideoPlayer, depth).
            /// Returns null if the file does not exist.
            /// </summary>
            protected abstract string GetLocalFilePath(string relativePath, string sessionId);

            /// <summary>Directory to delete when the session ends (extracted files), or null.</summary>
            protected abstract string TempDirForSession(string sessionId);

            public byte[] TryGetAttachmentBytes(string attachmentName)
            {
                if (s_SessionData == null ||
                    !s_SessionData.Attachments.TryGetValue(attachmentName, out var att) ||
                    string.IsNullOrEmpty(att.File))
                    return null;
                return ReadSessionFile(att.File);
            }

            public void StartLoading(SfzSessionData data, int bufSize, int framesToWait)
            {
                int totalFrames = data.FrameCount;
                if (totalFrames <= 0)
                {
                    Debug.LogError($"[SF] {BackendLabel}: frame_count is zero or missing.");
                    return;
                }

                string rgbPath = GetLocalFilePath(data.RgbMp4File, data.SessionId);
                if (rgbPath == null)
                {
                    Debug.LogError($"[SF] {BackendLabel}: color video '{data.RgbMp4File}' not found.");
                    return;
                }
                string depthBinPath = data.DepthBinFile != null
                    ? GetLocalFilePath(data.DepthBinFile, data.SessionId)
                    : null;

                // Parse frames.jsonl for poses, intrinsics, depth offsets and rgb_frame.
                var v2Records = ParseFramesJsonl(totalFrames);
                if (v2Records != null && v2Records.Length > 0 && v2Records.Length < totalFrames)
                    totalFrames = v2Records.Length;   // records without a color frame were removed

                bool hasFrames = data.Tracks.TryGetValue("frames", out var framesTrack);
                var vState = new VideoV2BackendState
                {
                    TotalFrames   = totalFrames,
                    FrameInterval = hasFrames ? framesTrack.SampleInterval : 1.0 / 60,
                };

                vState.AllocateRingBuffer();

                if (v2Records != null)
                {
                    for (int i = 0; i < v2Records.Length && i < totalFrames; i++)
                    {
                        vState.Poses[i]      = Matrix4x4.TRS(v2Records[i].Position, v2Records[i].Rotation, Vector3.one);
                        vState.Intrinsics[i] = v2Records[i].Intrinsics;
                    }
                }

                vState.InitializePlayers(rgbPath, depthBinPath, TempDirForSession(data.SessionId),
                    v2Records, data.RgbWidth, data.RgbHeight, data.DepthWidth, data.DepthHeight,
                    data.DepthEncoding, data.DepthUnitScale);

                m_State = vState;

                LatestTextureDimensions = new Vector2Int(data.RgbWidth, data.RgbHeight);
                Debug.Log($"[SF] {BackendLabel}: SFZ 2.x loading started. frames={totalFrames} fps={1.0 / vState.FrameInterval:F0}");
            }

            SfzV2FrameRecord[] ParseFramesJsonl(int expectedCount)
            {
                byte[] bytes = ReadSessionFile("frames.jsonl");
                if (bytes == null)
                {
                    Debug.LogError($"[SF] {BackendLabel}: frames.jsonl not found.");
                    return null;
                }

                try
                {
                    var records = new List<SfzV2FrameRecord>(expectedCount);
                    using var reader = new StreamReader(new MemoryStream(bytes), Encoding.UTF8);
                    string line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        if (string.IsNullOrWhiteSpace(line)) continue;
                        var j = JsonUtility.FromJson<SfzFrameLineJson>(line);
                        if (j == null) continue;
                        records.Add(new SfzV2FrameRecord
                        {
                            FrameIndex      = j.frame,
                            TimestampNs     = j.ts,
                            Position        = j.pos  != null && j.pos.Length >= 3
                                              ? new Vector3(j.pos[0], j.pos[1], j.pos[2])
                                              : Vector3.zero,
                            Rotation        = j.rot  != null && j.rot.Length >= 4
                                              ? new Quaternion(j.rot[0], j.rot[1], j.rot[2], j.rot[3])
                                              : Quaternion.identity,
                            Intrinsics      = new Vector4(j.fx, j.fy, j.cx, j.cy),
                            DepthByteOffset = j.depth_off,
                            DepthByteSize   = j.depth_sz,
                            RgbFrame        = j.rgb_frame,
                        });
                    }
                    Debug.Log($"[SF] {BackendLabel}: parsed {records.Count} frame records from frames.jsonl.");
                    return AlignToVideoFrames(records);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SF] {BackendLabel}: failed to parse frames.jsonl: {e}");
                    return null;
                }
            }

            // Playback is driven by rgb.mp4 frame indices, and poses/intrinsics/depth are looked
            // up by that index. When records carry rgb_frame, reorder them so record i is the one
            // shown with video frame i, dropping records whose color frame was not encoded.
            // Archives without rgb_frame keep the 1:1 assumption.
            static SfzV2FrameRecord[] AlignToVideoFrames(List<SfzV2FrameRecord> records)
            {
                if (records.Count == 0 || records.Exists(r => r.RgbFrame == RgbFrameAbsent))
                    return records.ToArray();

                var aligned = records.FindAll(r => r.RgbFrame >= 0);
                aligned.Sort((a, b) => a.RgbFrame.CompareTo(b.RgbFrame));

                int dropped = records.Count - aligned.Count;
                if (dropped > 0)
                    Debug.Log($"[SF] SFZ: {dropped} frame records have no color frame; aligned {aligned.Count} records to rgb.mp4.");
                for (int i = 0; i < aligned.Count; i++)
                    if (aligned[i].RgbFrame != i)
                    {
                        Debug.LogWarning($"[SF] SFZ: rgb_frame indices are not contiguous (record for video frame {i} has rgb_frame={aligned[i].RgbFrame}).");
                        break;
                    }
                return aligned.ToArray();
            }
        }

        // ── Nested: SFZ (ZIP archive) ─────────────────────────────────────────

        sealed class SfzFileBackend : SfzBackendBase
        {
            string m_ArchivePath;

            protected override string BackendLabel => "SFZ";

            public override bool Open(ARSensorFlexSession session)
            {
                m_Session     = session;
                m_ArchivePath = session.SfzFilePath;
                if (!Path.IsPathRooted(m_ArchivePath))
                    m_ArchivePath = Path.Combine(Application.streamingAssetsPath, m_ArchivePath);

                if (!File.Exists(m_ArchivePath))
                {
                    Debug.LogError($"[SF] SFZ archive not found: {m_ArchivePath}");
                    return false;
                }
                return true;
            }

            public override bool TryGetSessionJson(out string json)
            {
                json = null;
                byte[] bytes = ReadSessionFile("session.json");
                if (bytes == null) { Debug.LogError("[SF] SFZ: session/session.json not found."); return false; }
                json = Encoding.UTF8.GetString(bytes);
                return true;
            }

            protected override byte[] ReadSessionFile(string relativePath)
            {
                try
                {
                    using var archive = new ZipArchive(File.OpenRead(m_ArchivePath), ZipArchiveMode.Read);
                    var entry = archive.GetEntry("session/" + relativePath);
                    return entry != null ? ReadZipEntry(entry) : null;
                }
                catch (Exception e) { Debug.LogWarning($"[SF] SFZ: failed to read {relativePath}: {e.Message}"); return null; }
            }

            // Each session extracts into its own fresh directory, so files from a previous
            // (or crashed) session are never reused.
            protected override string TempDirForSession(string sessionId) =>
                Path.Combine(Application.temporaryCachePath, "SFVideoCache", sessionId);

            protected override string GetLocalFilePath(string relativePath, string sessionId)
            {
                if (string.IsNullOrEmpty(relativePath)) return null;
                string dir  = TempDirForSession(sessionId);
                string dest = Path.Combine(dir, relativePath);
                try
                {
                    using var archive = new ZipArchive(File.OpenRead(m_ArchivePath), ZipArchiveMode.Read);
                    var entry = archive.GetEntry("session/" + relativePath);
                    if (entry == null)
                    {
                        Debug.LogError($"[SF] SFZ: entry 'session/{relativePath}' not found.");
                        return null;
                    }
                    Directory.CreateDirectory(Path.GetDirectoryName(dest) ?? dir);
                    using (var src = entry.Open())
                    using (var dst = File.Create(dest))
                        src.CopyTo(dst);
                    Debug.Log($"[SF] Extracted {relativePath} ({entry.Length / 1024} KB) to {dest}");
                    return dest;
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SF] SFZ: failed to extract {relativePath}: {e}");
                    return null;
                }
            }

        }

        // ── Nested: FileIo (unpacked session directory) ───────────────────────

        sealed class FileIoBackend : SfzBackendBase
        {
            string m_SessionDir;

            protected override string BackendLabel => "FileIo";

            public override bool Open(ARSensorFlexSession session)
            {
                m_Session    = session;
                m_SessionDir = session.FileIoPath;
                if (!Path.IsPathRooted(m_SessionDir))
                    m_SessionDir = Path.Combine(Application.streamingAssetsPath, m_SessionDir);

                if (!Directory.Exists(m_SessionDir))
                {
                    Debug.LogError($"[SF] FileIo: session directory not found: {m_SessionDir}");
                    return false;
                }
                return true;
            }

            public override bool TryGetSessionJson(out string json)
            {
                json = null;
                string path = Path.Combine(m_SessionDir, "session.json");
                if (!File.Exists(path)) { Debug.LogError($"[SF] FileIo: session.json not found at {path}"); return false; }
                try   { json = File.ReadAllText(path); return true; }
                catch (Exception e) { Debug.LogError("[SF] FileIo: failed to read session.json: " + e); return false; }
            }

            protected override byte[] ReadSessionFile(string relativePath)
            {
                string fullPath = Path.Combine(m_SessionDir, relativePath);
                if (!File.Exists(fullPath)) return null;
                try   { return File.ReadAllBytes(fullPath); }
                catch (Exception e) { Debug.LogWarning($"[SF] FileIo: failed to read {relativePath}: {e.Message}"); return null; }
            }

            // Files are used in place and never deleted.
            protected override string TempDirForSession(string sessionId) => null;

            protected override string GetLocalFilePath(string relativePath, string sessionId)
            {
                if (string.IsNullOrEmpty(relativePath)) return null;
                string fullPath = Path.Combine(m_SessionDir, relativePath);
                if (File.Exists(fullPath)) return fullPath;
                Debug.LogError($"[SF] FileIo: {relativePath} not found in {m_SessionDir}");
                return null;
            }
        }

        // ── Nested: ScannedSceneMeshLoadOperation ─────────────────────────────

        sealed class ScannedSceneMeshLoadOperation
        {
            readonly Task<ScannedSceneMeshData> m_Task;
            readonly string m_SceneId;

            public bool   IsCompleted => m_Task.IsCompleted;
            public string SceneId     => m_SceneId;

            ScannedSceneMeshLoadOperation(Task<ScannedSceneMeshData> task, string sceneId)
            { m_Task = task; m_SceneId = sceneId; }

            public static ScannedSceneMeshLoadOperation StartFromPlyBytes(
                byte[] plyBytes, Matrix4x4 coordConvMatrix, string sceneId)
            {
                var task = Task.Run(() => PlyMeshReader.Parse(plyBytes, coordConvMatrix));
                return new ScannedSceneMeshLoadOperation(task, sceneId);
            }

            public bool TryComplete(out Mesh mesh)
            {
                mesh = null;
                if (!m_Task.IsCompleted) return false;

                if (m_Task.IsFaulted)
                {
                    Debug.LogError("[SF] Scanned mesh load failed: " + m_Task.Exception?.GetBaseException());
                    return true;
                }

                var data = m_Task.Result;
                if (data == null) return true;

                mesh = BuildUnityMesh(data, m_SceneId);
                return true;
            }

            static Mesh BuildUnityMesh(ScannedSceneMeshData data, string sceneId)
            {
                var mesh = new Mesh { name = $"SensorFlexScannedMesh-{sceneId}" };
                mesh.indexFormat = data.Vertices.Length > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16;
                mesh.vertices  = data.Vertices;
                mesh.triangles = data.Triangles;
                if (data.Colors  != null && data.Colors.Length  == data.Vertices.Length) mesh.colors32 = data.Colors;
                if (data.Normals != null && data.Normals.Length == data.Vertices.Length) mesh.normals  = data.Normals;
                else mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                return mesh;
            }
        }

        // ── Nested: ScannedSceneMeshData ──────────────────────────────────────

        sealed class ScannedSceneMeshData
        {
            public Vector3[] Vertices;
            public Vector3[] Normals;
            public Color32[] Colors;
            public int[]     Triangles;
        }

        // ── Nested: PlyMeshReader ─────────────────────────────────────────────

        static class PlyMeshReader
        {
            enum PlyFormat { Ascii, BinaryLittleEndian }

            sealed class Header
            {
                public PlyFormat Format;
                public int HeaderBytes;
                public int VertexCount;
                public int FaceCount;
                public readonly List<PlyProperty> VertexProperties = new();
                public PlyProperty FaceListProperty;
            }

            sealed class PlyProperty
            {
                public string Name;
                public string Type;
                public string CountType;
                public bool   IsList;
            }

            public static ScannedSceneMeshData Parse(byte[] bytes, Matrix4x4 coordConvMatrix)
            {
                var header = ParseHeader(bytes);
                return header.Format == PlyFormat.Ascii
                    ? ParseAscii(bytes, header, coordConvMatrix)
                    : ParseBinaryLittleEndian(bytes, header, coordConvMatrix);
            }

            static Header ParseHeader(byte[] bytes)
            {
                int headerEnd = FindHeaderEnd(bytes);
                string headerText = Encoding.ASCII.GetString(bytes, 0, headerEnd);
                var lines = headerText.Split('\n');
                var header = new Header { HeaderBytes = headerEnd };

                bool inVertexElement = false;
                bool inFaceElement   = false;
                foreach (var rawLine in lines)
                {
                    string line = rawLine.Trim();
                    if (string.IsNullOrEmpty(line)) continue;

                    if (line.StartsWith("format ", StringComparison.Ordinal))
                    {
                        if (line.Contains("ascii"))
                            header.Format = PlyFormat.Ascii;
                        else if (line.Contains("binary_little_endian"))
                            header.Format = PlyFormat.BinaryLittleEndian;
                        else
                            throw new NotSupportedException($"Unsupported PLY format line: {line}");
                    }
                    else if (line.StartsWith("element ", StringComparison.Ordinal))
                    {
                        inVertexElement = false;
                        inFaceElement   = false;
                        var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                        if (parts.Length < 3) continue;
                        if (parts[1] == "vertex")
                        { header.VertexCount = int.Parse(parts[2], CultureInfo.InvariantCulture); inVertexElement = true; }
                        else if (parts[1] == "face")
                        { header.FaceCount = int.Parse(parts[2], CultureInfo.InvariantCulture); inFaceElement = true; }
                    }
                    else if (line.StartsWith("property ", StringComparison.Ordinal))
                    {
                        var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                        if (inVertexElement)
                            header.VertexProperties.Add(new PlyProperty { Name = parts[^1], Type = parts[1] });
                        else if (inFaceElement && parts.Length >= 5 && parts[1] == "list")
                            header.FaceListProperty = new PlyProperty { IsList = true, CountType = parts[2], Type = parts[3], Name = parts[4] };
                    }
                }

                if (header.VertexCount <= 0)   throw new InvalidOperationException("PLY mesh has no vertices.");
                if (header.FaceCount   <= 0)   throw new InvalidOperationException("PLY mesh has no faces.");
                if (header.FaceListProperty == null) throw new NotSupportedException("PLY face list property is missing.");
                return header;
            }

            static int FindHeaderEnd(byte[] bytes)
            {
                var marker = Encoding.ASCII.GetBytes("end_header\n");
                int idx = IndexOf(bytes, marker);
                if (idx >= 0) return idx + marker.Length;

                marker = Encoding.ASCII.GetBytes("end_header\r\n");
                idx = IndexOf(bytes, marker);
                if (idx >= 0) return idx + marker.Length;

                throw new InvalidOperationException("PLY header terminator not found.");
            }

            static ScannedSceneMeshData ParseAscii(byte[] bytes, Header header, Matrix4x4 coordConvMatrix)
            {
                using var reader = new StringReader(Encoding.ASCII.GetString(bytes, header.HeaderBytes, bytes.Length - header.HeaderBytes));
                var    vertices = new Vector3[header.VertexCount];
                Vector3[] normals = TryAllocateNormals(header);
                Color32[] colors  = TryAllocateColors(header);

                for (int i = 0; i < header.VertexCount; i++)
                    vertices[i] = ReadVertex(ReadTokens(reader), header.VertexProperties, ref normals, ref colors, i);

                var triangles = new List<int>(header.FaceCount * 3);
                for (int i = 0; i < header.FaceCount; i++)
                {
                    var parts = ReadTokens(reader);
                    int count = int.Parse(parts[0], CultureInfo.InvariantCulture);
                    TriangulateFace(parts, 1, count, triangles);
                }

                ApplyCoordinateConversion(vertices, normals, triangles, coordConvMatrix);
                return new ScannedSceneMeshData { Vertices = vertices, Normals = normals, Colors = colors, Triangles = triangles.ToArray() };
            }

            static ScannedSceneMeshData ParseBinaryLittleEndian(byte[] bytes, Header header, Matrix4x4 coordConvMatrix)
            {
                var    vertices = new Vector3[header.VertexCount];
                Vector3[] normals = TryAllocateNormals(header);
                Color32[] colors  = TryAllocateColors(header);

                using var ms = new MemoryStream(bytes, header.HeaderBytes, bytes.Length - header.HeaderBytes, false);
                using var br = new BinaryReader(ms);

                for (int i = 0; i < header.VertexCount; i++)
                    vertices[i] = ReadBinaryVertex(br, header.VertexProperties, ref normals, ref colors, i);

                var triangles = new List<int>(header.FaceCount * 3);
                for (int i = 0; i < header.FaceCount; i++)
                {
                    int count   = (int)ReadScalar(br, header.FaceListProperty.CountType);
                    var indices = new int[count];
                    for (int j = 0; j < count; j++)
                        indices[j] = (int)ReadScalar(br, header.FaceListProperty.Type);
                    TriangulateFace(indices, triangles);
                }

                ApplyCoordinateConversion(vertices, normals, triangles, coordConvMatrix);
                return new ScannedSceneMeshData { Vertices = vertices, Normals = normals, Colors = colors, Triangles = triangles.ToArray() };
            }

            static string[] ReadTokens(StringReader reader)
            {
                string line;
                do
                {
                    line = reader.ReadLine();
                    if (line == null) throw new EndOfStreamException("Unexpected end of ASCII PLY stream.");
                    line = line.Trim();
                }
                while (line.Length == 0);
                return line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            }

            static Vector3[] TryAllocateNormals(Header header)
            {
                foreach (var p in header.VertexProperties)
                    if (p.Name == "nx" || p.Name == "ny" || p.Name == "nz")
                        return new Vector3[header.VertexCount];
                return null;
            }

            static Color32[] TryAllocateColors(Header header)
            {
                bool r = false, g = false, b = false;
                foreach (var p in header.VertexProperties)
                {
                    if (p.Name == "red")   r = true;
                    if (p.Name == "green") g = true;
                    if (p.Name == "blue")  b = true;
                }
                return r && g && b ? new Color32[header.VertexCount] : null;
            }

            static Vector3 ReadVertex(string[] parts, List<PlyProperty> props, ref Vector3[] normals, ref Color32[] colors, int idx)
            {
                float x = 0, y = 0, z = 0, nx = 0, ny = 0, nz = 0;
                byte red = 255, green = 255, blue = 255, alpha = 255;
                for (int i = 0; i < props.Count && i < parts.Length; i++)
                {
                    switch (props[i].Name)
                    {
                        case "x":     x     = float.Parse(parts[i], CultureInfo.InvariantCulture); break;
                        case "y":     y     = float.Parse(parts[i], CultureInfo.InvariantCulture); break;
                        case "z":     z     = float.Parse(parts[i], CultureInfo.InvariantCulture); break;
                        case "nx":    nx    = float.Parse(parts[i], CultureInfo.InvariantCulture); break;
                        case "ny":    ny    = float.Parse(parts[i], CultureInfo.InvariantCulture); break;
                        case "nz":    nz    = float.Parse(parts[i], CultureInfo.InvariantCulture); break;
                        case "red":   red   = byte.Parse(parts[i],  CultureInfo.InvariantCulture); break;
                        case "green": green = byte.Parse(parts[i],  CultureInfo.InvariantCulture); break;
                        case "blue":  blue  = byte.Parse(parts[i],  CultureInfo.InvariantCulture); break;
                        case "alpha": alpha = byte.Parse(parts[i],  CultureInfo.InvariantCulture); break;
                    }
                }
                if (normals != null) normals[idx] = new Vector3(nx, ny, nz);
                if (colors  != null) colors[idx]  = new Color32(red, green, blue, alpha);
                return new Vector3(x, y, z);
            }

            static Vector3 ReadBinaryVertex(BinaryReader br, List<PlyProperty> props, ref Vector3[] normals, ref Color32[] colors, int idx)
            {
                float x = 0, y = 0, z = 0, nx = 0, ny = 0, nz = 0;
                byte red = 255, green = 255, blue = 255, alpha = 255;
                for (int i = 0; i < props.Count; i++)
                {
                    switch (props[i].Name)
                    {
                        case "x":     x     = (float)ReadScalar(br, props[i].Type); break;
                        case "y":     y     = (float)ReadScalar(br, props[i].Type); break;
                        case "z":     z     = (float)ReadScalar(br, props[i].Type); break;
                        case "nx":    nx    = (float)ReadScalar(br, props[i].Type); break;
                        case "ny":    ny    = (float)ReadScalar(br, props[i].Type); break;
                        case "nz":    nz    = (float)ReadScalar(br, props[i].Type); break;
                        case "red":   red   = (byte)ReadScalar(br, props[i].Type);  break;
                        case "green": green = (byte)ReadScalar(br, props[i].Type);  break;
                        case "blue":  blue  = (byte)ReadScalar(br, props[i].Type);  break;
                        case "alpha": alpha = (byte)ReadScalar(br, props[i].Type);  break;
                        default:      ReadScalar(br, props[i].Type);                break;
                    }
                }
                if (normals != null) normals[idx] = new Vector3(nx, ny, nz);
                if (colors  != null) colors[idx]  = new Color32(red, green, blue, alpha);
                return new Vector3(x, y, z);
            }

            static double ReadScalar(BinaryReader br, string type) => type switch
            {
                "char"   or "int8"    => br.ReadSByte(),
                "uchar"  or "uint8"   => br.ReadByte(),
                "short"  or "int16"   => br.ReadInt16(),
                "ushort" or "uint16"  => br.ReadUInt16(),
                "int"    or "int32"   => br.ReadInt32(),
                "uint"   or "uint32"  => br.ReadUInt32(),
                "float"  or "float32" => br.ReadSingle(),
                "double" or "float64" => br.ReadDouble(),
                _ => throw new NotSupportedException($"Unsupported PLY scalar type '{type}'.")
            };

            static void TriangulateFace(string[] parts, int start, int count, List<int> triangles)
            {
                if (count < 3) return;
                int first = int.Parse(parts[start], CultureInfo.InvariantCulture);
                for (int i = 1; i < count - 1; i++)
                {
                    triangles.Add(first);
                    triangles.Add(int.Parse(parts[start + i],     CultureInfo.InvariantCulture));
                    triangles.Add(int.Parse(parts[start + i + 1], CultureInfo.InvariantCulture));
                }
            }

            static void TriangulateFace(int[] indices, List<int> triangles)
            {
                if (indices.Length < 3) return;
                int first = indices[0];
                for (int i = 1; i < indices.Length - 1; i++)
                {
                    triangles.Add(first);
                    triangles.Add(indices[i]);
                    triangles.Add(indices[i + 1]);
                }
            }

            static void ApplyCoordinateConversion(Vector3[] vertices, Vector3[] normals, List<int> triangles, Matrix4x4 m)
            {
                for (int i = 0; i < vertices.Length; i++)
                    vertices[i] = m.MultiplyPoint3x4(vertices[i]);
                if (normals != null)
                    for (int i = 0; i < normals.Length; i++)
                        normals[i] = m.MultiplyVector(normals[i]).normalized;
                if (ChangesHandedness(m))
                    for (int i = 0; i + 2 < triangles.Count; i += 3)
                        (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
            }

            static bool ChangesHandedness(Matrix4x4 m)
            {
                var x = m.MultiplyVector(Vector3.right);
                var y = m.MultiplyVector(Vector3.up);
                var z = m.MultiplyVector(Vector3.forward);
                return Vector3.Dot(Vector3.Cross(x, y), z) < 0f;
            }

            static int IndexOf(byte[] haystack, byte[] needle)
            {
                for (int i = 0; i <= haystack.Length - needle.Length; i++)
                {
                    bool match = true;
                    for (int j = 0; j < needle.Length; j++)
                        if (haystack[i + j] != needle[j]) { match = false; break; }
                    if (match) return i;
                }
                return -1;
            }
        }

        // ── Nested: ScannedMeshLoaderImpl ─────────────────────────────────────

        sealed class ScannedMeshLoaderImpl
        {
            // SFZ attachments are already in the Unity world frame of the camera poses
            // (SensorFlexFormat.md); converters do any source-frame conversion.
            static readonly Matrix4x4 k_AttachmentToUnity = Matrix4x4.identity;

            ScannedSceneMeshLoadOperation m_PendingLoad;
            bool m_Started;

            public bool IsComplete
            {
                get
                {
                    var data = s_SessionData;
                    if (data == null) return false;
                    if (!data.Attachments.ContainsKey("scene_mesh")) return true;
                    return m_Started && m_PendingLoad == null;
                }
            }

            public void Tick()
            {
                var sessionData = SfzSessionStore.s_SessionData;
                if (sessionData == null) return;

                if (!m_Started && sessionData.Attachments.ContainsKey("scene_mesh"))
                {
                    var bytes = SfzSessionStore.TryConsumeAttachment("scene_mesh");
                    if (bytes != null)
                    {
                        m_PendingLoad = ScannedSceneMeshLoadOperation.StartFromPlyBytes(
                            bytes, k_AttachmentToUnity, sessionData.SessionId);
                        m_Started = true;
                        Debug.Log("[SF] SfzSessionStore: scene_mesh loading started.");
                    }
                }

                if (m_PendingLoad == null) return;
                if (!m_PendingLoad.TryComplete(out var mesh)) return;

                if (mesh != null)
                {
                    ScannedSceneMeshBridge.SetMesh(mesh, m_PendingLoad.SceneId);
                    Debug.Log($"[SF] Mesh ready: vertices={mesh.vertexCount} triangles={mesh.triangles.Length / 3}");
                }
                m_PendingLoad = null;
            }

            public void Reset()
            {
                m_PendingLoad = null;
                m_Started     = false;
            }
        }
    }
}
