// SfzSessionStore.cs — session store for SFZ (ZIP archive) and FileIo (loose file) sessions.
//
// Single source of truth for SFZ-mode session lifecycle and data access.
// CameraSubsystem drives it through StartSession / Tick / StopSessionAsync;
// OcclusionSubsystem, ControlBridge, and other consumers read state through
// the proxy properties — never touching IO types directly.
//
// Private nested types (all invisible outside this file):
//   IBackendState / BackendState  — ring-buffer state contract + implementation
//   ISessionBackend               — three-phase backend contract (Open / TryGetSessionJson / StartLoading)
//   SessionLoadState              — session lifecycle enum (Idle / Waiting / LoadingAttachments / Loading / Ready)
//   SfzTrackInfo                  — per-track metadata (name, sample interval, record count)
//   SfzAttachmentInfo             — per-attachment metadata (name, file, format)
//   SfzSessionData                — parsed session.json aggregate
//   SfzSessionJson / DTOs         — session.json deserialization types
//   SfzPoseToMatrix4x4            — SFZ pose record → Matrix4x4
//   SfzIntrinsicsToVector4        — SFZ intrinsics record → Vector4
//   ReadZipEntry                  — drains a ZipArchiveEntry into byte[]
//   SfzBackendBase                — ring-buffer streaming on a background thread
//   SfzFileBackend                — SFZ (ZIP archive) source
//   FileIoBackend                 — FileIo (loose files) source
//   ScannedMeshLoaderImpl         — polls for scene_mesh attachment, drives PLY parse
//   ScannedSceneMeshLoadOperation — wraps Task-based PLY parse; TryComplete() polled each frame
//   ScannedSceneMeshData          — parsed vertex/normal/color/triangle arrays
//   PlyMeshReader                 — ASCII and binary-little-endian PLY decoder

using System;
using System.Collections.Concurrent;
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
        internal static RenderTexture VideoDepthTexture      => s_Backend?.State?.VideoDepthTexture;
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
                    if (s_Backend.TryGetSessionJson(out var json) && TryParseSession(json, out s_SessionData))
                    {
                        s_LoadState = SessionLoadState.LoadingAttachments;
                        Debug.Log($"[SF] SfzSessionStore: session parsed. id={s_SessionData.SessionId} " +
                                  $"tracks={s_SessionData.Tracks.Count} attachments={s_SessionData.Attachments.Count}");
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

        static bool TryParseSession(string json, out SfzSessionData data)
        {
            data = null;
            if (string.IsNullOrEmpty(json)) return false;

            var raw = JsonUtility.FromJson<SfzSessionJson>(json);
            if (raw == null) return false;

            // Dispatch v2.0 format.
            if (raw.sfz_version == "2.0")
                return TryParseSessionV2(json, out data);

            if (string.IsNullOrEmpty(raw.version)) return false;

            var tracks = new Dictionary<string, SfzTrackInfo>();
            if (raw.tracks?.frames != null)
            {
                int fps   = raw.tracks.frames.metadata?.fps ?? 30;
                int count = raw.tracks.frames.data?.Length ?? 0;
                tracks["frames"] = new SfzTrackInfo("frames", 1.0 / Math.Max(1, fps), count);
            }
            if (raw.tracks?.imu != null)
            {
                float hz  = raw.tracks.imu.metadata?.sample_rate_hz ?? 100f;
                int count = raw.tracks.imu.data?.Length ?? 0;
                tracks["imu"] = new SfzTrackInfo("imu", 1.0 / Math.Max(1f, hz), count);
            }

            var attachments = new Dictionary<string, SfzAttachmentInfo>();
            if (raw.attachments?.scene_mesh != null)
                attachments["scene_mesh"] = new SfzAttachmentInfo(
                    "scene_mesh",
                    raw.attachments.scene_mesh.file,
                    raw.attachments.scene_mesh.format);

            var rgbCh   = raw.tracks?.frames?.metadata?.channels?.rgb;
            var depthCh = raw.tracks?.frames?.metadata?.channels?.depth;
            bool isVideo = rgbCh?.format == "hevc_mp4";

            data = new SfzSessionData(
                sessionId:    raw.session_id,
                tracks:       tracks,
                attachments:  attachments,
                frameRecords: raw.tracks?.frames?.data,
                isVideoFormat: isVideo,
                isV2Format:   false,
                rgbMp4File:   isVideo ? (rgbCh?.file   ?? "rgb.mp4")   : null,
                depthMp4File: isVideo ? (depthCh?.file ?? "depth.mp4") : null,
                depthBinFile: null,
                rgbWidth:    rgbCh?.width    ?? 1920,
                rgbHeight:   rgbCh?.height   ?? 1440,
                depthWidth:  depthCh?.width  ?? 256,
                depthHeight: depthCh?.height ?? 192);

            return true;
        }

        static bool TryParseSessionV2(string json, out SfzSessionData data)
        {
            data = null;
            var raw = JsonUtility.FromJson<SfzV2SessionJson>(json);
            if (raw == null || string.IsNullOrEmpty(raw.session_id)) return false;

            double sampleInterval = raw.actual_fps > 0 ? 1.0 / raw.actual_fps : 1.0 / 60;
            var tracks = new Dictionary<string, SfzTrackInfo>
            {
                ["frames"] = new SfzTrackInfo("frames", sampleInterval, raw.frame_count)
            };

            // Dispatch on depth encoding to determine which file to use.
            string depthEncoding = raw.depth?.encoding ?? "lz4_float32";
            string depthBinFile  = null;
            string depthMp4File  = null;
            switch (depthEncoding)
            {
                case "hevc_float16":
                case "hevc_uint8_norm":
                case "h264_uint8_norm":
                    depthMp4File = raw.depth?.file ?? "depth.mp4";
                    break;
                default: // "lz4_float32", "lz4_uint16", "zstd_float32" and unknown
                    depthBinFile = raw.depth?.file ?? "depth.bin";
                    break;
            }

            // Factor that converts a decoded depth value to metres.
            float depthUnitScale = raw.depth?.units switch
            {
                "millimeters" => 0.001f,
                "centimeters" => 0.01f,
                _             => 1f,
            };

            float depthMaxMeters = raw.depth != null && raw.depth.depth_max_meters > 0f
                ? raw.depth.depth_max_meters : 10f;

            data = new SfzSessionData(
                raw.session_id,
                tracks,
                new Dictionary<string, SfzAttachmentInfo>(),
                frameRecords:   null,
                isVideoFormat:  false,
                isV2Format:     true,
                rgbMp4File:     raw.rgb?.file   ?? "rgb.mp4",
                depthMp4File:   depthMp4File,
                depthBinFile:   depthBinFile,
                depthEncoding:  depthEncoding,
                depthMaxMeters: depthMaxMeters,
                depthUnitScale: depthUnitScale,
                rgbWidth:       raw.rgb?.width    ?? 1920,
                rgbHeight:      raw.rgb?.height   ?? 1440,
                depthWidth:     raw.depth?.width  ?? 256,
                depthHeight:    raw.depth?.height ?? 192,
                frameCount:     raw.frame_count);

            return true;
        }

        // ── Nested: Ring buffer state ─────────────────────────────────────────

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
            RenderTexture VideoDepthTexture  { get; }
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

        sealed class BackendState : IBackendState
        {
            int m_PlayHead;
            int m_BufferedFrames;

            public double    FrameInterval                 { get; set; }
            public Matrix4x4 CoordConvMatrix               { get; set; } = Matrix4x4.identity;
            public bool      UseNegativeZForwardOpticalAxis { get; set; }
            public int       TotalFrames                   { get; set; } = int.MaxValue;
            public int       BufSize                       { get; }
            public bool      IsReady                       { get; set; }
            public Texture2D[]  Frames      { get; set; }
            public byte[][]     DepthBins   { get; set; }
            public Matrix4x4[]  Poses       { get; set; }
            public Vector4[]    Intrinsics  { get; set; }
            public bool[]       SlotReady   { get; set; }
            public int[]        SlotGlobalIdx { get; set; }
            public int PendingDecodeCount   { get; set; }

            public int PlayHead
            {
                get => Volatile.Read(ref m_PlayHead);
                set => Volatile.Write(ref m_PlayHead, value);
            }

            int m_LatestGlobalIndex = -1;

            public int LatestGlobalIndex
            {
                get => Volatile.Read(ref m_LatestGlobalIndex);
                set => Volatile.Write(ref m_LatestGlobalIndex, value);
            }

            public BackendState(int bufSize) { BufSize = bufSize; }

            public void AllocateRingBuffer()
            {
                Frames        = new Texture2D[BufSize];
                DepthBins     = new byte[BufSize][];
                Poses         = new Matrix4x4[BufSize];
                Intrinsics    = new Vector4[BufSize];
                SlotReady     = new bool[BufSize];
                SlotGlobalIdx = new int[BufSize];

                for (int i = 0; i < BufSize; i++)
                    SlotGlobalIdx[i] = -1;
            }

            public void MarkBuffered(int framesToWait)
            {
                m_BufferedFrames++;
                if (!IsReady && m_BufferedFrames >= framesToWait)
                    IsReady = true;
            }

            public void DestroyTextures()
            {
                if (Frames == null) return;
                for (int i = 0; i < Frames.Length; i++)
                {
                    if (Frames[i] == null) continue;
                    UnityEngine.Object.Destroy(Frames[i]);
                    Frames[i] = null;
                }
            }

            public bool          IsVideoMode              => false;
            public RenderTexture VideoRgbTexture          => null;
            public RenderTexture VideoDepthTexture        => null;
            public void          SeekVideoFrame(int fi)  { }
            public long          VideoPlayerCurrentFrame  => 0;
            public void          PlayVideo()              { }
            public void          PauseVideo()             { }
            public void          SetVideoPlaybackSpeed(float s) { }
            public void          SetVideoLooping(bool loop)     { }
            public void          SetFrameReadyCallback(Action<int> cb) { }
            public void          UpdateDepthForFrame(int fi) { }
            public Texture2D     V2DepthTexture               => null;
        }

        // ── Nested: Video backend state (v1.1) ───────────────────────────────

        sealed class VideoBackendState : IBackendState
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

            // ── Video-specific ────────────────────────────────────────────────
            public bool IsVideoMode => true;

            RenderTexture m_RgbRT;
            RenderTexture m_DepthRT;
            GameObject    m_RgbGo;
            GameObject    m_DepthGo;
            VideoPlayer   m_RgbPlayer;
            VideoPlayer   m_DepthPlayer;
            bool          m_RgbPrepared;
            bool          m_DepthPrepared;
            string        m_TempRgbPath;
            string        m_TempDepthPath;
            long          m_CurrentFrame = -1; // updated by frameReady event, not .frame getter
            Action<int>   m_FrameReadyCallback;

            public RenderTexture VideoRgbTexture   => m_RgbRT;
            public RenderTexture VideoDepthTexture => m_DepthRT;

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

            public void InitializePlayers(
                string rgbPath, string depthPath,
                int rgbW, int rgbH, int depthW, int depthH)
            {
                m_TempRgbPath   = rgbPath;
                m_TempDepthPath = depthPath;

                m_RgbRT   = new RenderTexture(rgbW, rgbH, 0, RenderTextureFormat.ARGB32) { name = "SF-RgbVideo" };
                m_DepthRT = new RenderTexture(depthW, depthH, 0, RenderTextureFormat.ARGB32) { name = "SF-DepthBgraVideo" };
                m_RgbRT.Create();
                m_DepthRT.Create();

                m_RgbGo   = new GameObject("SF-VideoPlayer-RGB")   { hideFlags = HideFlags.HideAndDontSave };
                m_DepthGo = new GameObject("SF-VideoPlayer-Depth") { hideFlags = HideFlags.HideAndDontSave };
                UnityEngine.Object.DontDestroyOnLoad(m_RgbGo);
                UnityEngine.Object.DontDestroyOnLoad(m_DepthGo);

                m_RgbPlayer = m_RgbGo.AddComponent<VideoPlayer>();
                ConfigurePlayer(m_RgbPlayer, "file://" + rgbPath, m_RgbRT);
                m_RgbPlayer.sendFrameReadyEvents = true;
                m_RgbPlayer.frameReady          += OnRgbFrameReady;
                m_RgbPlayer.prepareCompleted    += OnRgbPrepared;
                m_RgbPlayer.Prepare();

                m_DepthPlayer = m_DepthGo.AddComponent<VideoPlayer>();
                ConfigurePlayer(m_DepthPlayer, "file://" + depthPath, m_DepthRT);
                m_DepthPlayer.prepareCompleted += OnDepthPrepared;
                m_DepthPlayer.Prepare();
            }

            static void ConfigurePlayer(VideoPlayer player, string url, RenderTexture rt)
            {
                player.source          = VideoSource.Url;
                player.url             = url;
                player.renderMode      = VideoRenderMode.RenderTexture;
                player.targetTexture   = rt;
                player.playOnAwake     = false;
                player.isLooping       = false;
                player.audioOutputMode = VideoAudioOutputMode.None;
                player.skipOnDrop      = false;
            }

            public void SetFrameReadyCallback(Action<int> cb) { m_FrameReadyCallback = cb; }

            void OnRgbFrameReady(VideoPlayer _, long frameIdx)
            {
                if (frameIdx <= 3 || frameIdx % 300 == 0)
                    Debug.Log($"[SF] frameReady: frameIdx={frameIdx} TotalFrames={TotalFrames}");
                Volatile.Write(ref m_CurrentFrame, frameIdx);
                m_FrameReadyCallback?.Invoke((int)frameIdx);
            }

            void OnRgbPrepared(VideoPlayer _)   { m_RgbPrepared   = true; CheckBothPrepared(); }
            void OnDepthPrepared(VideoPlayer _) { m_DepthPrepared = true; CheckBothPrepared(); }

            void CheckBothPrepared()
            {
                if (!m_RgbPrepared || !m_DepthPrepared) return;
                // Stop auto-play that some platforms trigger after Prepare despite playOnAwake=false.
                // This prevents the video from advancing before the warmup transition runs.
                if (m_RgbPlayer.isPlaying)   m_RgbPlayer.Pause();
                if (m_DepthPlayer.isPlaying) m_DepthPlayer.Pause();
                IsReady = true;
                Debug.Log("[SF] VideoBackendState: both VideoPlayers prepared.");
            }

            // Called by the camera subsystem after warmup to start natural playback.
            public void PlayVideo()
            {
                m_RgbPlayer?.Play();
                m_DepthPlayer?.Play();
            }

            public void PauseVideo()
            {
                if (m_RgbPlayer?.isPlaying   == true) m_RgbPlayer.Pause();
                if (m_DepthPlayer?.isPlaying == true) m_DepthPlayer.Pause();
            }

            public void SetVideoPlaybackSpeed(float speed)
            {
                if (m_RgbPlayer   != null) m_RgbPlayer.playbackSpeed   = speed;
                if (m_DepthPlayer != null) m_DepthPlayer.playbackSpeed = speed;
            }

            public void SetVideoLooping(bool loop)
            {
                if (m_RgbPlayer   != null) m_RgbPlayer.isLooping   = loop;
                if (m_DepthPlayer != null) m_DepthPlayer.isLooping = loop;
            }

            // Current frame — updated via frameReady event, not the unreliable .frame getter.
            public long VideoPlayerCurrentFrame => Volatile.Read(ref m_CurrentFrame);

            // Frame seek is only used for step-forward while paused; never during normal play.
            public void SeekVideoFrame(int frameIndex)
            {
                if (m_RgbPlayer   != null && m_RgbPlayer.isPrepared)   m_RgbPlayer.frame   = (long)frameIndex;
                if (m_DepthPlayer != null && m_DepthPlayer.isPrepared) m_DepthPlayer.frame = (long)frameIndex;
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
                if (m_DepthPlayer != null)
                {
                    m_DepthPlayer.prepareCompleted -= OnDepthPrepared;
                    m_DepthPlayer.Stop();
                    m_DepthPlayer = null;
                }
                if (m_RgbGo   != null) { UnityEngine.Object.Destroy(m_RgbGo);   m_RgbGo   = null; }
                if (m_DepthGo != null) { UnityEngine.Object.Destroy(m_DepthGo); m_DepthGo = null; }
                if (m_RgbRT   != null) { m_RgbRT.Release();   UnityEngine.Object.Destroy(m_RgbRT);   m_RgbRT   = null; }
                if (m_DepthRT != null) { m_DepthRT.Release(); UnityEngine.Object.Destroy(m_DepthRT); m_DepthRT = null; }
                TryDeleteTemp(m_TempRgbPath);
                TryDeleteTemp(m_TempDepthPath);
            }

            static void TryDeleteTemp(string path)
            {
                if (string.IsNullOrEmpty(path)) return;
                try { if (File.Exists(path)) File.Delete(path); }
                catch (Exception e) { Debug.LogWarning($"[SF] Failed to delete temp file {path}: {e.Message}"); }
            }

            public void      UpdateDepthForFrame(int fi) { }
            public Texture2D V2DepthTexture               => null;
        }

        // ── Nested: Video backend state (v2.0) ───────────────────────────────
        // RGB VideoPlayer only; depth loaded on-demand from extracted depth.bin
        // via K4os LZ4_RAW decompression into a RFloat Texture2D.

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

            // ── Video-specific ────────────────────────────────────────────────
            public bool IsVideoMode => true;

            RenderTexture m_RgbRT;
            GameObject    m_RgbGo;
            VideoPlayer   m_RgbPlayer;
            bool          m_RgbPrepared;
            string        m_TempRgbPath;
            string        m_TempDepthBinPath;
            long          m_CurrentFrame = -1;
            Action<int>   m_FrameReadyCallback;

            // ── Bin Depth (LZ4 or Zstd) ───────────────────────────────────────
            SfzV2FrameRecord[] m_V2Records;
            FileStream         m_DepthStream;
            Texture2D          m_DepthTexture;
            int                m_DepthW, m_DepthH;
            byte[]             m_CompressedBuf;
            byte[]             m_DecompressedBuf;
            string             m_DepthEncoding;  // "lz4_float32", "lz4_uint16" or "zstd_float32"
            float              m_DepthUnitScale = 1f;
            byte[]             m_U16Buf;         // lz4_uint16: decoded uint16 before conversion to float metres

#if UNITY_IOS && !UNITY_EDITOR
            [System.Runtime.InteropServices.DllImport("__Internal")]
            static extern UIntPtr SFDecodeBlock(IntPtr src, UIntPtr srcLen,
                                                IntPtr dst, UIntPtr dstLen, int algo);
#endif

            // ── Video Depth (optional) ─────────────────────────────────────────
            bool          m_UseDepthVideo;
            RenderTexture m_DepthRT;
            GameObject    m_DepthGo;
            VideoPlayer   m_DepthVideoPlayer;
            bool          m_DepthPrepared;
            string        m_TempDepthVideoPath;

            public RenderTexture VideoRgbTexture   => m_RgbRT;
            public RenderTexture VideoDepthTexture => m_UseDepthVideo ? m_DepthRT : null;
            public Texture2D     V2DepthTexture    => m_UseDepthVideo ? null : m_DepthTexture;

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

            public void InitializePlayers(
                string rgbPath, string depthBinPath,
                SfzV2FrameRecord[] v2Records,
                int rgbW, int rgbH, int depthW, int depthH,
                string depthVideoPath = null,
                string depthEncoding  = "lz4_float32",
                float  depthUnitScale = 1f)
            {
                m_TempRgbPath      = rgbPath;
                m_TempDepthBinPath = depthBinPath;
                m_V2Records        = v2Records;
                m_DepthW           = depthW;
                m_DepthH           = depthH;
                m_UseDepthVideo    = !string.IsNullOrEmpty(depthVideoPath);
                m_DepthEncoding    = depthEncoding ?? "lz4_float32";
                m_DepthUnitScale   = depthUnitScale > 0f ? depthUnitScale : 1f;

                m_RgbRT = new RenderTexture(rgbW, rgbH, 0, RenderTextureFormat.ARGB32) { name = "SF-V2-RgbVideo" };
                m_RgbRT.Create();

                if (m_UseDepthVideo)
                {
                    // Video depth mode — create a VideoPlayer for depth.mp4.
                    m_TempDepthVideoPath = depthVideoPath;
                    m_DepthRT = new RenderTexture(depthW, depthH, 0, RenderTextureFormat.ARGB32) { name = "SF-V2-DepthVideo" };
                    m_DepthRT.Create();

                    m_DepthGo = new GameObject("SF-VideoPlayer-V2-Depth") { hideFlags = HideFlags.HideAndDontSave };
                    UnityEngine.Object.DontDestroyOnLoad(m_DepthGo);

                    m_DepthVideoPlayer = m_DepthGo.AddComponent<VideoPlayer>();
                    m_DepthVideoPlayer.source          = VideoSource.Url;
                    m_DepthVideoPlayer.url             = "file://" + depthVideoPath;
                    m_DepthVideoPlayer.renderMode      = VideoRenderMode.RenderTexture;
                    m_DepthVideoPlayer.targetTexture   = m_DepthRT;
                    m_DepthVideoPlayer.playOnAwake     = false;
                    m_DepthVideoPlayer.isLooping       = false;
                    m_DepthVideoPlayer.audioOutputMode = VideoAudioOutputMode.None;
                    m_DepthVideoPlayer.skipOnDrop      = false;
                    m_DepthVideoPlayer.prepareCompleted += OnDepthVideoPrepared;
                    m_DepthVideoPlayer.Prepare();
                }
                else
                {
                    // LZ4 depth mode — open FileStream on depth.bin.
                    m_DepthTexture = new Texture2D(depthW, depthH, TextureFormat.RFloat, false) { name = "SF-V2-Depth" };

                    if (!string.IsNullOrEmpty(depthBinPath) && File.Exists(depthBinPath))
                        m_DepthStream = File.OpenRead(depthBinPath);
                    else if (!string.IsNullOrEmpty(depthBinPath))
                        Debug.LogWarning($"[SF] V2: depth.bin not found at {depthBinPath}");

                    m_DecompressedBuf = new byte[depthW * depthH * 4];
                    if (m_DepthEncoding == "lz4_uint16")
                        m_U16Buf = new byte[depthW * depthH * 2];
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
                m_RgbPrepared = true;
                CheckAllPrepared();
            }

            void OnDepthVideoPrepared(VideoPlayer _)
            {
                m_DepthPrepared = true;
                CheckAllPrepared();
            }

            void CheckAllPrepared()
            {
                if (!m_RgbPrepared) return;
                if (m_UseDepthVideo && !m_DepthPrepared) return;
                if (m_RgbPlayer.isPlaying) m_RgbPlayer.Pause();
                if (m_UseDepthVideo && m_DepthVideoPlayer != null && m_DepthVideoPlayer.isPlaying)
                    m_DepthVideoPlayer.Pause();
                IsReady = true;
                Debug.Log("[SF] VideoV2BackendState: prepared. depthVideo=" + m_UseDepthVideo);
            }

            public void UpdateDepthForFrame(int frameIdx)
            {
                // No-op when depth is a VideoPlayer — it advances automatically.
                if (m_UseDepthVideo) return;

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
                        DecodeZstdBlock(m_CompressedBuf, rec.DepthByteSize, m_DecompressedBuf);
                    else if (m_DepthEncoding == "lz4_uint16")
                    {
                        Lz4BlockDecoder.Decode(m_CompressedBuf, 0, rec.DepthByteSize,
                                               m_U16Buf, 0, m_U16Buf.Length);
                        U16ToFloatMeters(m_U16Buf, m_DecompressedBuf, m_DepthUnitScale);
                    }
                    else
                        Lz4BlockDecoder.Decode(m_CompressedBuf, 0, rec.DepthByteSize,
                                               m_DecompressedBuf, 0, m_DecompressedBuf.Length);
                    m_DepthTexture.LoadRawTextureData(m_DecompressedBuf);
                    m_DepthTexture.Apply(false);
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[SF] V2 depth decode failed frame={frameIdx}: {e.Message}");
                }
            }

            // Little-endian uint16 → float32 (value × scale) for the RFloat depth texture.
            static void U16ToFloatMeters(byte[] src, byte[] dst, float scale)
            {
                var s16 = System.Runtime.InteropServices.MemoryMarshal.Cast<byte, ushort>(src);
                var f32 = System.Runtime.InteropServices.MemoryMarshal.Cast<byte, float>(dst);
                int n = Math.Min(s16.Length, f32.Length);
                for (int i = 0; i < n; i++)
                    f32[i] = s16[i] * scale;
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

            public void PlayVideo()
            {
                m_RgbPlayer?.Play();
                if (m_UseDepthVideo) m_DepthVideoPlayer?.Play();
            }

            public void PauseVideo()
            {
                if (m_RgbPlayer?.isPlaying == true) m_RgbPlayer.Pause();
                if (m_UseDepthVideo && m_DepthVideoPlayer?.isPlaying == true) m_DepthVideoPlayer.Pause();
            }

            public void SetVideoPlaybackSpeed(float speed)
            {
                if (m_RgbPlayer != null) m_RgbPlayer.playbackSpeed = speed;
                if (m_UseDepthVideo && m_DepthVideoPlayer != null) m_DepthVideoPlayer.playbackSpeed = speed;
            }

            public void SetVideoLooping(bool loop)
            {
                if (m_RgbPlayer != null) m_RgbPlayer.isLooping = loop;
                if (m_UseDepthVideo && m_DepthVideoPlayer != null) m_DepthVideoPlayer.isLooping = loop;
            }

            public long VideoPlayerCurrentFrame => Volatile.Read(ref m_CurrentFrame);

            public void SeekVideoFrame(int frameIndex)
            {
                if (m_RgbPlayer != null && m_RgbPlayer.isPrepared)
                    m_RgbPlayer.frame = (long)frameIndex;
                if (m_UseDepthVideo && m_DepthVideoPlayer != null && m_DepthVideoPlayer.isPrepared)
                    m_DepthVideoPlayer.frame = (long)frameIndex;
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

                if (m_UseDepthVideo)
                {
                    if (m_DepthVideoPlayer != null)
                    {
                        m_DepthVideoPlayer.prepareCompleted -= OnDepthVideoPrepared;
                        m_DepthVideoPlayer.Stop();
                        m_DepthVideoPlayer = null;
                    }
                    if (m_DepthGo != null) { UnityEngine.Object.Destroy(m_DepthGo); m_DepthGo = null; }
                    if (m_DepthRT != null) { m_DepthRT.Release(); UnityEngine.Object.Destroy(m_DepthRT); m_DepthRT = null; }
                    TryDeleteTemp(m_TempDepthVideoPath);
                }
                else
                {
                    if (m_DepthTexture != null) { UnityEngine.Object.Destroy(m_DepthTexture); m_DepthTexture = null; }
                    m_DepthStream?.Close();
                    m_DepthStream = null;
                    TryDeleteTemp(m_TempDepthBinPath);
                }

                TryDeleteTemp(m_TempRgbPath);
            }

            static void TryDeleteTemp(string path)
            {
                if (string.IsNullOrEmpty(path)) return;
                try { if (File.Exists(path)) File.Delete(path); }
                catch (Exception e) { Debug.LogWarning($"[SF] Failed to delete temp file {path}: {e.Message}"); }
            }
        }

        // ── Nested: SFZ v2.0 frame record ────────────────────────────────────

        struct SfzV2FrameRecord
        {
            public int        FrameIndex;
            public long       TimestampNs;
            public Vector3    Position;
            public Quaternion Rotation;
            public Vector4    Intrinsics;    // (fx, fy, cx, cy)
            public long       DepthByteOffset;
            public int        DepthByteSize;
            public int        RgbFrame;      // index into rgb.mp4; -1 = no color; RgbFrameAbsent = old archive
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

            /// <summary>Allocate ring buffer and start streaming. Called once after parse.</summary>
            void StartLoading(SfzSessionData data, int bufSize, int framesToWait);

            /// <summary>
            /// Returns raw bytes for a named attachment when available, null otherwise.
            /// Calling this consumes the bytes (subsequent calls return null for the same name).
            /// </summary>
            byte[] TryGetAttachmentBytes(string attachmentName);

            /// <summary>Ring-buffer state; non-null after StartLoading.</summary>
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
            internal SfzFrameRecordJson[] FrameRecords { get; }
            internal bool   IsVideoFormat { get; }
            internal bool   IsV2Format    { get; }
            internal string RgbMp4File    { get; }
            internal string DepthMp4File  { get; }
            internal string DepthBinFile  { get; }
            internal string DepthEncoding { get; }  // e.g. "lz4_float32", "hevc_float16", "h264_uint8_norm"
            internal float  DepthMaxMeters { get; } // for uint8_norm codecs
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
                SfzFrameRecordJson[]                           frameRecords,
                bool   isVideoFormat   = false,
                bool   isV2Format      = false,
                string rgbMp4File      = null,
                string depthMp4File    = null,
                string depthBinFile    = null,
                string depthEncoding   = "lz4_float32",
                float  depthMaxMeters  = 10f,
                float  depthUnitScale  = 1f,
                int    rgbWidth        = 1920,
                int    rgbHeight       = 1440,
                int    depthWidth      = 256,
                int    depthHeight     = 192,
                int    frameCount      = 0)
            {
                SessionId     = sessionId   ?? "session";
                Tracks        = tracks      ?? new Dictionary<string, SfzTrackInfo>();
                Attachments   = attachments ?? new Dictionary<string, SfzAttachmentInfo>();
                FrameRecords  = frameRecords;
                IsVideoFormat  = isVideoFormat;
                IsV2Format    = isV2Format;
                RgbMp4File    = rgbMp4File;
                DepthMp4File  = depthMp4File;
                DepthBinFile  = depthBinFile;
                DepthEncoding  = depthEncoding ?? "lz4_float32";
                DepthMaxMeters = depthMaxMeters > 0f ? depthMaxMeters : 10f;
                DepthUnitScale = depthUnitScale > 0f ? depthUnitScale : 1f;
                RgbWidth      = rgbWidth;
                RgbHeight     = rgbHeight;
                DepthWidth    = depthWidth;
                DepthHeight   = depthHeight;
                FrameCount    = frameCount;
            }
        }

        // ── Nested: session.json DTOs ─────────────────────────────────────────

        [Serializable] class SfzDeviceJson { public string model; public string os; public string ar_framework; }

        [Serializable] class SfzRgbChannelJson   { public int width; public int height; public string format; public string file; }
        [Serializable] class SfzDepthChannelJson { public int width; public int height; public string format; public string file; public float invalid_value; }
        [Serializable] class SfzChannelsJson     { public SfzRgbChannelJson rgb; public SfzDepthChannelJson depth; }

        [Serializable] class SfzFramesMetadataJson { public int fps; public SfzChannelsJson channels; }

        [Serializable] class SfzFileRefJson         { public string file; public int frame_index; }
        [Serializable] class SfzPoseJson            { public float[] position; public float[] rotation; }
        [Serializable] class SfzIntrinsicsJson      { public float fx; public float fy; public float cx; public float cy; }
        [Serializable] class SfzLightEstimationJson { public float ambient_intensity; public float color_temperature; }
        [Serializable] class SfzCameraJson          { public SfzPoseJson pose; public SfzIntrinsicsJson intrinsics; }
        [Serializable] class SfzFrameRecordJson
        {
            public long                   timestamp_ns;
            public SfzCameraJson          camera;
            public SfzLightEstimationJson light_estimation;
            public SfzFileRefJson         rgb;
            public SfzFileRefJson         depth;
        }
        [Serializable] class SfzFramesTrackJson { public SfzFramesMetadataJson metadata; public SfzFrameRecordJson[] data; }

        [Serializable] class SfzImuMetadataJson { public float sample_rate_hz; }
        [Serializable] class SfzImuSampleJson
        {
            public long    timestamp_ns;
            public float[] acceleration;
            public float[] rotation_rate;
            public float[] gravity;
        }
        [Serializable] class SfzImuTrackJson { public SfzImuMetadataJson metadata; public SfzImuSampleJson[] data; }

        [Serializable] class SfzTracksJson    { public SfzFramesTrackJson frames; public SfzImuTrackJson imu; }

        [Serializable] class SfzMeshAttachmentJson { public string file; public string format; }
        [Serializable] class SfzAttachmentsJson    { public SfzMeshAttachmentJson scene_mesh; }

        [Serializable] class SfzPartContentJson
        {
            public string type;
            public string attachment_key;
            public int    chunk_index;
            public int    total_chunks;
            public int[]  frame_range;
        }
        [Serializable] class SfzPartJson { public string file; public SfzPartContentJson[] contents; }

        [Serializable] class SfzSessionJson
        {
            public string              sfz_version;   // "2.0" for v2.0 files
            public string              version;       // "1.0" / "1.1" for legacy files
            public string              session_id;
            public string              start_time_utc;
            public SfzDeviceJson       device;
            public SfzTracksJson       tracks;
            public SfzAttachmentsJson  attachments;
            public SfzPartJson[]       parts;
        }

        // ── Nested: SFZ v2.0 DTOs ────────────────────────────────────────────

        [Serializable] class SfzV2RgbJson   { public int width; public int height; public string encoding; public string file; }
        [Serializable] class SfzV2DepthJson { public int width; public int height; public string encoding; public string file; public string units; public string sensor; public float depth_max_meters; }
        [Serializable] class SfzV2SessionJson
        {
            public string         sfz_version;
            public string         session_id;
            public string         start_time_utc;
            public SfzDeviceJson  device;
            public int            actual_fps;
            public int            frame_count;
            public SfzV2RgbJson   rgb;
            public SfzV2DepthJson depth;
        }

        [Serializable] class SfzV2FrameLineJson
        {
            public int   frame;
            public long  ts;
            public float[] pos;
            public float[] rot;
            public float fx, fy, cx, cy;
            public long  depth_off;
            public int   depth_sz;
            public int   depth_frame;
            public int   rgb_frame = RgbFrameAbsent;   // absent in archives written before rgb_frame existed
        }

        // ── Nested: SFZ conversion helpers ───────────────────────────────────

        static Matrix4x4 SfzPoseToMatrix4x4(SfzPoseJson pose)
        {
            if (pose?.position == null || pose.position.Length < 3 ||
                pose.rotation  == null || pose.rotation.Length  < 4)
                return Matrix4x4.identity;
            return Matrix4x4.TRS(
                new Vector3(pose.position[0], pose.position[1], pose.position[2]),
                new Quaternion(pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]),
                Vector3.one);
        }

        static Vector4 SfzIntrinsicsToVector4(SfzIntrinsicsJson intr) =>
            intr != null ? new Vector4(intr.fx, intr.fy, intr.cx, intr.cy) : Vector4.zero;

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

        // ── Nested: SfzBackendBase ────────────────────────────────────────────

        abstract class SfzBackendBase : ISessionBackend
        {
            const int UploadBatchSize = 3;

            // Frame packet exchanged between the background loader thread and the main thread.
            struct LoadedFrame
            {
                public int    GlobalFrameIndex;
                public int    RecordIndex;
                public byte[] Jpg;
                public byte[] DepthBin;
            }

            protected ARSensorFlexSession m_Session;
            protected IBackendState       m_State;
            int           m_FramesToWait;
            volatile bool m_StopLoading;
            Thread        m_LoadThread;
            ConcurrentQueue<LoadedFrame> m_UploadQueue;
            int  m_UploadedFrames;
            bool m_LoggedFirstEnqueue;
            bool m_LoggedFirstUpload;
            bool m_LoggedReady;

            protected SfzSessionData m_SessionData;

            // ── ISessionBackend ───────────────────────────────────────────────

            public abstract bool   Open(ARSensorFlexSession session);
            public abstract bool   TryGetSessionJson(out string json);
            public abstract byte[] TryGetAttachmentBytes(string attachmentName);

            public virtual void StartLoading(SfzSessionData data, int bufSize, int framesToWait)
            {
                m_SessionData        = data;
                m_FramesToWait       = framesToWait;
                m_StopLoading        = false;
                m_UploadedFrames     = 0;
                m_LoggedFirstEnqueue = false;
                m_LoggedFirstUpload  = false;
                m_LoggedReady        = false;

                if (m_SessionData.FrameRecords == null || m_SessionData.FrameRecords.Length == 0)
                {
                    Debug.LogError($"[SF] {BackendLabel}: frames track missing or empty.");
                    return;
                }

                m_State = new BackendState(bufSize);

                bool hasFrames = m_SessionData.Tracks.TryGetValue("frames", out var framesTrack);
                m_State.TotalFrames    = hasFrames && framesTrack.RecordCount > 0
                    ? framesTrack.RecordCount
                    : int.MaxValue;
                m_State.FrameInterval  = hasFrames ? framesTrack.SampleInterval : 1.0 / 30;
                m_State.CoordConvMatrix               = Matrix4x4.identity;
                m_State.UseNegativeZForwardOpticalAxis = false;
                m_State.AllocateRingBuffer();

                m_UploadQueue = new ConcurrentQueue<LoadedFrame>();
                m_LoadThread  = new Thread(LoadFrames) { IsBackground = true, Name = $"SF-{BackendLabel}" };
                m_LoadThread.Start();

                Debug.Log($"[SF] {BackendLabel} streaming started. frames={m_State.TotalFrames} fps={1.0 / m_State.FrameInterval:F0} bufSize={m_State.BufSize}");
            }

            public IBackendState State    => m_State;
            public void          Dispatch() { }

            public Task StopAsync()
            {
                m_StopLoading = true;
                if (m_LoadThread != null && m_LoadThread.IsAlive)
                {
                    m_LoadThread.Join(500);
                    m_LoadThread = null;
                }
                m_UploadQueue = null;
                return Task.CompletedTask;
            }

            public void DrainMainThreadWork()
            {
                if (m_UploadQueue == null) return;

                int uploaded = 0;
                while (uploaded < UploadBatchSize && m_UploadQueue.TryDequeue(out var item))
                {
                    int slot = item.GlobalFrameIndex % m_State.BufSize;

                    if (m_State.Frames[slot] == null)
                        m_State.Frames[slot] = new Texture2D(2, 2, TextureFormat.RGBA32, false);

                    m_State.Frames[slot].LoadImage(item.Jpg);
                    m_State.Frames[slot].Apply();
                    m_State.DepthBins[slot] = item.DepthBin;

                    var records = m_SessionData?.FrameRecords;
                    if (records != null && item.RecordIndex >= 0 && item.RecordIndex < records.Length)
                    {
                        var rec = records[item.RecordIndex];
                        if (rec.camera?.pose != null)
                            m_State.Poses[slot] = SfzPoseToMatrix4x4(rec.camera.pose);
                        if (rec.camera?.intrinsics != null)
                            m_State.Intrinsics[slot] = SfzIntrinsicsToVector4(rec.camera.intrinsics);
                    }

                    m_State.SlotGlobalIdx[slot] = item.GlobalFrameIndex;
                    m_State.SlotReady[slot]     = true;
                    m_State.MarkBuffered(m_FramesToWait);
                    m_UploadedFrames++;

                    if (!m_LoggedFirstUpload)
                    {
                        Debug.Log($"[SF] {BackendLabel} first frame uploaded. GlobalFrame={item.GlobalFrameIndex} Slot={slot}");
                        m_LoggedFirstUpload = true;
                    }

                    if (!m_LoggedReady && m_State.IsReady)
                    {
                        Debug.Log($"[SF] {BackendLabel} ready. Uploaded={m_UploadedFrames} FramesToWait={m_FramesToWait}");
                        m_LoggedReady = true;
                    }

                    uploaded++;
                }
            }

            // ── Subclass contract ─────────────────────────────────────────────

            protected abstract string BackendLabel { get; }
            protected abstract byte[] ReadSessionFile(string relativePath);
            protected virtual  void   BeginLoadPass() { }
            protected virtual  void   EndLoadPass()   { }

            // ── Background loader thread ──────────────────────────────────────

            void LoadFrames()
            {
                bool looping   = m_Session.LoopSequence;
                int  iteration = 0;

                while (!m_StopLoading)
                {
                    int globalOffset = iteration * m_State.TotalFrames;
                    BeginLoadPass();
                    try
                    {
                        for (int i = 0; i < m_SessionData.FrameRecords.Length && !m_StopLoading; i++)
                        {
                            var record = m_SessionData.FrameRecords[i];
                            if (string.IsNullOrEmpty(record.rgb?.file)) continue;

                            byte[] jpg = ReadSessionFile(record.rgb.file);
                            if (jpg == null) continue;

                            byte[] depth = !string.IsNullOrEmpty(record.depth?.file)
                                ? ReadSessionFile(record.depth.file)
                                : null;

                            Enqueue(globalOffset + i, i, jpg, depth);
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.LogError($"[SF] {BackendLabel} loader error: {ex}");
                        return;
                    }
                    finally
                    {
                        EndLoadPass();
                    }
                    if (!looping) break;
                    iteration++;
                }
            }

            void Enqueue(int globalFrameIndex, int recordIndex, byte[] jpg, byte[] depth)
            {
                while (!m_StopLoading && globalFrameIndex - m_State.PlayHead >= m_State.BufSize)
                    Thread.Sleep(1);

                if (m_StopLoading) return;

                if (!m_LoggedFirstEnqueue)
                {
                    Debug.Log($"[SF] {BackendLabel} first frame enqueued. GlobalFrame={globalFrameIndex}");
                    m_LoggedFirstEnqueue = true;
                }

                m_UploadQueue.Enqueue(new LoadedFrame
                {
                    GlobalFrameIndex = globalFrameIndex,
                    RecordIndex      = recordIndex,
                    Jpg              = jpg,
                    DepthBin         = depth
                });
            }
        }

        // ── Nested: SFZ (ZIP archive) ─────────────────────────────────────────

        sealed class SfzFileBackend : SfzBackendBase
        {
            string     m_ArchivePath;    // single-file path, or part-0 path in multi-part
            string[]   m_PartPaths;      // null in single-file mode
            ZipArchive   m_PassArchive;  // single-file pass archive (background thread)
            ZipArchive[] m_PassArchives; // multi-part pass archives (background thread)

            // Built from the "parts" manifest in session.json (multi-part only)
            (int Start, int End, int PartIdx)[]                       m_FrameRanges;
            Dictionary<string, List<(int ChunkIdx, int PartIdx)>>     m_AttachChunks;

            protected override string BackendLabel => "SFZ";

            public override bool Open(ARSensorFlexSession session)
            {
                m_Session  = session;
                string path = session.SfzFilePath;
                if (!Path.IsPathRooted(path))
                    path = Path.Combine(Application.streamingAssetsPath, path);

                if (TryResolveMultiPart(path, out var partPaths))
                {
                    m_PartPaths   = partPaths;
                    m_ArchivePath = partPaths[0];
                    foreach (var p in partPaths)
                    {
                        if (!File.Exists(p))
                        {
                            Debug.LogError($"[SF] SFZ multi-part: missing part file: {p}");
                            return false;
                        }
                    }
                    Debug.Log($"[SF] SFZ multi-part: {partPaths.Length} parts detected and all present.");
                }
                else
                {
                    m_PartPaths   = null;
                    m_ArchivePath = path;
                    if (!File.Exists(m_ArchivePath))
                    {
                        Debug.LogError($"[SF] SFZ archive not found: {m_ArchivePath}");
                        return false;
                    }
                }
                return true;
            }

            public override bool TryGetSessionJson(out string json)
            {
                json = null;
                try
                {
                    // session.json is always in part 0 (m_ArchivePath points there for both modes)
                    using var archive = new ZipArchive(File.OpenRead(m_ArchivePath), ZipArchiveMode.Read);
                    var entry = archive.GetEntry("session/session.json");
                    if (entry == null) { Debug.LogError("[SF] SFZ: session/session.json not found."); return false; }
                    using var sr = new StreamReader(entry.Open());
                    json = sr.ReadToEnd();

                    if (m_PartPaths != null)
                        BuildPartManifest(json);

                    return true;
                }
                catch (Exception e) { Debug.LogError("[SF] SFZ: failed to read session.json: " + e); return false; }
            }

            public override byte[] TryGetAttachmentBytes(string attachmentName)
            {
                if (s_SessionData == null ||
                    !s_SessionData.Attachments.TryGetValue(attachmentName, out var att) ||
                    string.IsNullOrEmpty(att.File))
                    return null;

                if (m_PartPaths == null || m_AttachChunks == null ||
                    !m_AttachChunks.TryGetValue(attachmentName, out var chunks))
                {
                    // Single-file or no manifest — read from m_ArchivePath (part 0 in multi-part)
                    try
                    {
                        using var archive = new ZipArchive(File.OpenRead(m_ArchivePath), ZipArchiveMode.Read);
                        var entry = archive.GetEntry($"session/{att.File}");
                        return entry != null ? ReadZipEntry(entry) : null;
                    }
                    catch (Exception e) { Debug.LogWarning($"[SF] SFZ: failed to read attachment '{attachmentName}': {e.Message}"); return null; }
                }

                // Multi-part: read and concatenate chunks in order
                var chunkBuffers = new List<byte[]>(chunks.Count);
                long totalLen = 0;
                foreach (var (_, partIdx) in chunks)
                {
                    try
                    {
                        using var archive = new ZipArchive(File.OpenRead(m_PartPaths[partIdx]), ZipArchiveMode.Read);
                        var entry = archive.GetEntry($"session/{att.File}");
                        if (entry == null)
                        {
                            Debug.LogWarning($"[SF] SFZ: attachment chunk missing in part {partIdx}: {att.File}");
                            return null;
                        }
                        var b = ReadZipEntry(entry);
                        chunkBuffers.Add(b);
                        totalLen += b.Length;
                    }
                    catch (Exception e) { Debug.LogWarning($"[SF] SFZ: failed to read attachment chunk '{attachmentName}' part {partIdx}: {e.Message}"); return null; }
                }

                if (chunkBuffers.Count == 1) return chunkBuffers[0];

                var combined = new byte[totalLen];
                int offset = 0;
                foreach (var b in chunkBuffers) { Buffer.BlockCopy(b, 0, combined, offset, b.Length); offset += b.Length; }
                return combined;
            }

            // ── Video and v2.0 loading ────────────────────────────────────────

            public override void StartLoading(SfzSessionData data, int bufSize, int framesToWait)
            {
                if (data.IsV2Format)
                {
                    StartV2Loading(data);
                    return;
                }
                if (data.IsVideoFormat)
                {
                    StartVideoLoading(data);
                    return;
                }
                base.StartLoading(data, bufSize, framesToWait);
            }

            void StartVideoLoading(SfzSessionData data)
            {
                int totalFrames = data.FrameRecords?.Length ?? 0;
                if (totalFrames == 0)
                {
                    Debug.LogError("[SF] SFZ video: no frame records found.");
                    return;
                }

                bool hasFrames = data.Tracks.TryGetValue("frames", out var framesTrack);

                string cacheDir = Path.Combine(Application.temporaryCachePath, "SFVideoCache");
                Directory.CreateDirectory(cacheDir);

                string rgbPath   = ExtractMp4ToTemp(data.RgbMp4File,   cacheDir);
                string depthPath = ExtractMp4ToTemp(data.DepthMp4File, cacheDir);
                if (rgbPath == null || depthPath == null) return;

                var vState = new VideoBackendState
                {
                    TotalFrames   = totalFrames,
                    FrameInterval = hasFrames ? framesTrack.SampleInterval : 1.0 / 60,
                };

                vState.AllocateRingBuffer();

                for (int i = 0; i < data.FrameRecords.Length; i++)
                {
                    var rec = data.FrameRecords[i];
                    if (rec.camera?.pose       != null) vState.Poses[i]      = SfzPoseToMatrix4x4(rec.camera.pose);
                    if (rec.camera?.intrinsics != null) vState.Intrinsics[i] = SfzIntrinsicsToVector4(rec.camera.intrinsics);
                }

                vState.InitializePlayers(rgbPath, depthPath,
                    data.RgbWidth, data.RgbHeight,
                    data.DepthWidth, data.DepthHeight);

                m_State       = vState;
                m_SessionData = data;

                LatestTextureDimensions = new Vector2Int(data.RgbWidth, data.RgbHeight);
                Debug.Log($"[SF] SFZ video loading started. frames={totalFrames} fps={1.0 / vState.FrameInterval:F0}");
            }

            void StartV2Loading(SfzSessionData data)
            {
                int totalFrames = data.FrameCount;
                if (totalFrames <= 0)
                {
                    Debug.LogError("[SF] SFZ v2: frame_count is zero or missing.");
                    return;
                }

                bool hasFrames = data.Tracks.TryGetValue("frames", out var framesTrack);

                string cacheDir = Path.Combine(Application.temporaryCachePath, "SFVideoCache");
                Directory.CreateDirectory(cacheDir);

                string rgbPath        = ExtractFileToTemp(data.RgbMp4File,   cacheDir);
                string depthBinPath   = ExtractFileToTemp(data.DepthBinFile,  cacheDir);
                string depthVideoPath = ExtractFileToTemp(data.DepthMp4File,  cacheDir);
                if (rgbPath == null) return;

                // Parse frames.jsonl for poses, intrinsics, and depth offsets.
                var v2Records = ParseFramesJsonl(cacheDir, totalFrames);
                if (v2Records != null && v2Records.Length > 0 && v2Records.Length < totalFrames)
                    totalFrames = v2Records.Length;   // records without a color frame were removed

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

                vState.InitializePlayers(rgbPath, depthBinPath ?? string.Empty,
                    v2Records, data.RgbWidth, data.RgbHeight, data.DepthWidth, data.DepthHeight,
                    depthVideoPath: depthVideoPath,
                    depthEncoding:  data.DepthEncoding,
                    depthUnitScale: data.DepthUnitScale);

                m_State       = vState;
                m_SessionData = data;

                LatestTextureDimensions = new Vector2Int(data.RgbWidth, data.RgbHeight);
                Debug.Log($"[SF] SFZ v2.0 loading started. frames={totalFrames} fps={1.0 / vState.FrameInterval:F0}");
            }

            // Reads frames.jsonl from the SFZ archive into the cache dir, then parses it.
            SfzV2FrameRecord[] ParseFramesJsonl(string cacheDir, int expectedCount)
            {
                // frames.jsonl may already have been extracted, or extract now.
                string jsonlPath = Path.Combine(cacheDir, "frames.jsonl");
                if (!File.Exists(jsonlPath))
                {
                    try
                    {
                        using var archive = new ZipArchive(File.OpenRead(m_ArchivePath), ZipArchiveMode.Read);
                        var entry = archive.GetEntry("session/frames.jsonl");
                        if (entry == null)
                        {
                            Debug.LogError("[SF] SFZ v2: session/frames.jsonl not found in archive.");
                            return null;
                        }
                        using var src = entry.Open();
                        using var dst = File.Create(jsonlPath);
                        src.CopyTo(dst);
                    }
                    catch (Exception e)
                    {
                        Debug.LogError($"[SF] SFZ v2: failed to extract frames.jsonl: {e}");
                        return null;
                    }
                }

                try
                {
                    var records = new List<SfzV2FrameRecord>(expectedCount);
                    using var reader = new StreamReader(jsonlPath, Encoding.UTF8);
                    string line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        if (string.IsNullOrWhiteSpace(line)) continue;
                        var j = JsonUtility.FromJson<SfzV2FrameLineJson>(line);
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
                    Debug.Log($"[SF] SFZ v2: parsed {records.Count} frame records from frames.jsonl.");
                    return AlignToVideoFrames(records);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SF] SFZ v2: failed to parse frames.jsonl: {e}");
                    return null;
                }
            }

            // Playback is driven by rgb.mp4 frame indices, and poses/intrinsics/depth are looked
            // up by that index. When records carry rgb_frame, reorder them so record i is the one
            // shown with video frame i, dropping records whose color frame was not encoded.
            // Archives without rgb_frame keep the old 1:1 assumption.
            static SfzV2FrameRecord[] AlignToVideoFrames(List<SfzV2FrameRecord> records)
            {
                if (records.Count == 0 || records.Exists(r => r.RgbFrame == RgbFrameAbsent))
                    return records.ToArray();

                var aligned = records.FindAll(r => r.RgbFrame >= 0);
                aligned.Sort((a, b) => a.RgbFrame.CompareTo(b.RgbFrame));

                int dropped = records.Count - aligned.Count;
                if (dropped > 0)
                    Debug.Log($"[SF] SFZ v2: {dropped} frame records have no color frame; aligned {aligned.Count} records to rgb.mp4.");
                for (int i = 0; i < aligned.Count; i++)
                    if (aligned[i].RgbFrame != i)
                    {
                        Debug.LogWarning($"[SF] SFZ v2: rgb_frame indices are not contiguous (record for video frame {i} has rgb_frame={aligned[i].RgbFrame}).");
                        break;
                    }
                return aligned.ToArray();
            }

            string ExtractFileToTemp(string archiveFileName, string cacheDir)
            {
                if (string.IsNullOrEmpty(archiveFileName)) return null;
                string dest = Path.Combine(cacheDir, archiveFileName);
                if (File.Exists(dest)) return dest;   // already extracted
                try
                {
                    using var archive = new ZipArchive(File.OpenRead(m_ArchivePath), ZipArchiveMode.Read);
                    var entry = archive.GetEntry("session/" + archiveFileName);
                    if (entry == null)
                    {
                        Debug.LogError($"[SF] SFZ: entry 'session/{archiveFileName}' not found.");
                        return null;
                    }
                    using var src = entry.Open();
                    using var dst = File.Create(dest);
                    src.CopyTo(dst);
                    Debug.Log($"[SF] Extracted {archiveFileName} ({entry.Length / 1024} KB) to {dest}");
                    return dest;
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SF] SFZ: failed to extract {archiveFileName}: {e}");
                    return null;
                }
            }

            string ExtractMp4ToTemp(string mp4File, string cacheDir)
            {
                if (string.IsNullOrEmpty(mp4File)) return null;
                try
                {
                    using var archive = new ZipArchive(File.OpenRead(m_ArchivePath), ZipArchiveMode.Read);
                    var entry = archive.GetEntry("session/" + mp4File);
                    if (entry == null)
                    {
                        Debug.LogError($"[SF] SFZ: entry 'session/{mp4File}' not found.");
                        return null;
                    }
                    string dest = Path.Combine(cacheDir, mp4File);
                    using var src = entry.Open();
                    using var dst = File.Create(dest);
                    src.CopyTo(dst);
                    Debug.Log($"[SF] Extracted {mp4File} ({entry.Length / 1024} KB) to {dest}");
                    return dest;
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SF] SFZ: failed to extract {mp4File}: {e}");
                    return null;
                }
            }

            // Keep archives open for the duration of each pass so central directories
            // are parsed only once per loop iteration.
            protected override void BeginLoadPass()
            {
                if (m_PartPaths == null)
                {
                    m_PassArchive = new ZipArchive(File.OpenRead(m_ArchivePath), ZipArchiveMode.Read);
                }
                else
                {
                    m_PassArchives = new ZipArchive[m_PartPaths.Length];
                    for (int i = 0; i < m_PartPaths.Length; i++)
                        m_PassArchives[i] = new ZipArchive(File.OpenRead(m_PartPaths[i]), ZipArchiveMode.Read);
                }
            }

            protected override void EndLoadPass()
            {
                m_PassArchive?.Dispose();
                m_PassArchive = null;
                if (m_PassArchives != null)
                {
                    foreach (var a in m_PassArchives) a?.Dispose();
                    m_PassArchives = null;
                }
            }

            protected override byte[] ReadSessionFile(string relativePath)
            {
                try
                {
                    ZipArchive archive;
                    if (m_PartPaths == null)
                    {
                        archive = m_PassArchive;
                    }
                    else
                    {
                        int partIdx = FindPartForRelativePath(relativePath);
                        if (partIdx < 0) return null;
                        archive = m_PassArchives?[partIdx];
                    }
                    var entry = archive?.GetEntry($"session/{relativePath}");
                    return entry != null ? ReadZipEntry(entry) : null;
                }
                catch (Exception e) { Debug.LogWarning($"[SF] SFZ: failed to read {relativePath}: {e.Message}"); return null; }
            }

            // ── Multi-part helpers ────────────────────────────────────────────

            // Detects "basename-DDDDD-of-DDDDD.sfz", derives all part paths regardless
            // of which part index the given path represents.
            static bool TryResolveMultiPart(string resolvedPath, out string[] partPaths)
            {
                partPaths = null;
                string stem = Path.GetFileNameWithoutExtension(resolvedPath);
                string ext  = Path.GetExtension(resolvedPath);
                string dir  = Path.GetDirectoryName(resolvedPath) ?? string.Empty;

                int ofIdx = stem.LastIndexOf("-of-", StringComparison.Ordinal);
                if (ofIdx < 0) return false;

                string afterOf  = stem[(ofIdx + 4)..];
                string beforeOf = stem[..ofIdx];
                int dashIdx = beforeOf.LastIndexOf('-');
                if (dashIdx < 0) return false;

                if (!int.TryParse(beforeOf[(dashIdx + 1)..], out _) ||
                    !int.TryParse(afterOf, out int total) || total <= 0)
                    return false;

                string baseName = beforeOf[..dashIdx];
                partPaths = new string[total];
                for (int i = 0; i < total; i++)
                    partPaths[i] = Path.Combine(dir, $"{baseName}-{i:D5}-of-{total:D5}{ext}");
                return true;
            }

            void BuildPartManifest(string json)
            {
                var raw = JsonUtility.FromJson<SfzSessionJson>(json);
                if (raw?.parts == null || raw.parts.Length == 0) return;

                var frameRanges = new List<(int Start, int End, int PartIdx)>();
                m_AttachChunks  = new Dictionary<string, List<(int ChunkIdx, int PartIdx)>>();

                foreach (var part in raw.parts)
                {
                    if (part.contents == null) continue;
                    int partIdx = Array.FindIndex(m_PartPaths, p => Path.GetFileName(p) == part.file);
                    if (partIdx < 0) continue;

                    foreach (var content in part.contents)
                    {
                        if (content.type == "frames" && content.frame_range?.Length >= 2)
                        {
                            frameRanges.Add((content.frame_range[0], content.frame_range[1], partIdx));
                        }
                        else if (content.type == "attachment" && !string.IsNullOrEmpty(content.attachment_key))
                        {
                            if (!m_AttachChunks.TryGetValue(content.attachment_key, out var list))
                                m_AttachChunks[content.attachment_key] = list = new List<(int, int)>();
                            list.Add((content.chunk_index, partIdx));
                        }
                    }
                }

                frameRanges.Sort((a, b) => a.Start.CompareTo(b.Start));
                m_FrameRanges = frameRanges.ToArray();
                foreach (var kvp in m_AttachChunks)
                    kvp.Value.Sort((a, b) => a.ChunkIdx.CompareTo(b.ChunkIdx));

                Debug.Log($"[SF] SFZ multi-part manifest: {m_FrameRanges.Length} frame range(s), {m_AttachChunks.Count} attachment type(s).");
            }

            int FindPartForRelativePath(string relativePath)
            {
                if (m_FrameRanges == null || m_FrameRanges.Length == 0) return 0;
                string stem = Path.GetFileNameWithoutExtension(relativePath);
                if (!int.TryParse(stem, NumberStyles.None, CultureInfo.InvariantCulture, out int frameIdx))
                    return -1;
                foreach (var (start, end, partIdx) in m_FrameRanges)
                    if (frameIdx >= start && frameIdx < end) return partIdx;
                return -1;
            }
        }

        // ── Nested: FileIo (loose files) ──────────────────────────────────────

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

            public override byte[] TryGetAttachmentBytes(string attachmentName)
            {
                if (s_SessionData == null ||
                    !s_SessionData.Attachments.TryGetValue(attachmentName, out var att) ||
                    string.IsNullOrEmpty(att.File))
                    return null;

                string fullPath = Path.Combine(m_SessionDir, att.File);
                if (!File.Exists(fullPath)) return null;
                try   { return File.ReadAllBytes(fullPath); }
                catch (Exception e) { Debug.LogWarning($"[SF] FileIo: failed to read attachment '{attachmentName}': {e.Message}"); return null; }
            }

            protected override byte[] ReadSessionFile(string relativePath)
            {
                string fullPath = Path.Combine(m_SessionDir, relativePath);
                if (!File.Exists(fullPath)) return null;
                try   { return File.ReadAllBytes(fullPath); }
                catch (Exception e) { Debug.LogWarning($"[SF] FileIo: failed to read {relativePath}: {e.Message}"); return null; }
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
            // ScanNet++ PLY meshes are in ARKit space (right-handed, -Z forward).
            // Flip Z to land positions in the same Unity world space as camera poses.
            static readonly Matrix4x4 k_ArkitToUnity = new Matrix4x4(
                new Vector4( 1, 0,  0, 0),
                new Vector4( 0, 1,  0, 0),
                new Vector4( 0, 0, -1, 0),
                new Vector4( 0, 0,  0, 1));

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
                            bytes, k_ArkitToUnity, sessionData.SessionId);
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
