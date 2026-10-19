using System;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR.ARSubsystems;
using SensorFlex.Player.Library;

namespace SensorFlex.Player.Subsystem
{
    /// <summary>
    /// XROcclusionSubsystem provider that serves environment depth from the active
    /// SfzSessionStore ring buffer. Supported for Sfz and FileIo modes.
    ///</summary>
    public sealed class OcclusionSubsystem : XROcclusionSubsystem
    {
        const string SubsystemId = "SensorFlex-Occlusion";

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void RegisterDescriptor()
        {
            XROcclusionSubsystemDescriptor.Register(new XROcclusionSubsystemDescriptor.Cinfo
            {
                id = SubsystemId,
                providerType = typeof(DepthDataProvider),
                subsystemTypeOverride = typeof(OcclusionSubsystem),
                // Human segmentation not supported — omit delegates (null → Unsupported)
                humanSegmentationStencilImageSupportedDelegate = null,
                humanSegmentationDepthImageSupportedDelegate   = null,
                // Environment depth is supported
                environmentDepthImageSupportedDelegate             = () => Supported.Supported,
                environmentDepthConfidenceImageSupportedDelegate   = null,
                environmentDepthTemporalSmoothingSupportedDelegate = null,
            });
            Debug.Log("[SF] Occlusion Subsystem registered.");
        }

        class DepthDataProvider : Provider
        {
            const int RawDepthWidth = 256;
            const int RawDepthHeight = 192;
            static readonly int EnvironmentDepthPropertyId = Shader.PropertyToID("_EnvironmentDepth");

            ARSensorFlexSession session;
            Texture m_CurrentDepthTexture;
            bool framesReady;
            bool m_HasBoundSession;
            bool m_LoggedWaitingForSession;
            float m_CurrentDepthWorldScale = 1f;
            bool m_IsSfzMode;
            Texture2D m_SfzDepthTexture;
            int m_LastSfzPlayHead = -1;

            // ----------------------------------------------------------------
            // Environment depth mode
            // ----------------------------------------------------------------
            EnvironmentDepthMode m_RequestedDepthMode = EnvironmentDepthMode.Fastest;

            public override EnvironmentDepthMode requestedEnvironmentDepthMode
            {
                get => m_RequestedDepthMode;
                set => m_RequestedDepthMode = value;
            }

            public override EnvironmentDepthMode currentEnvironmentDepthMode
                => framesReady ? m_RequestedDepthMode : EnvironmentDepthMode.Disabled;

            // ----------------------------------------------------------------
            // Occlusion preference
            // ----------------------------------------------------------------
            OcclusionPreferenceMode m_RequestedOcclusionMode = OcclusionPreferenceMode.PreferEnvironmentOcclusion;

            public override OcclusionPreferenceMode requestedOcclusionPreferenceMode
            {
                get => m_RequestedOcclusionMode;
                set => m_RequestedOcclusionMode = value;
            }

            public override OcclusionPreferenceMode currentOcclusionPreferenceMode
                => m_RequestedOcclusionMode;

            // ----------------------------------------------------------------
            // Lifecycle
            // ----------------------------------------------------------------
            public override void Start()
            {
                CameraSubsystem.CameraDataProvider.OnFramesReady += AdvanceFrame;
                m_HasBoundSession = false;
                m_LoggedWaitingForSession = false;
            }

            public override void Stop()
            {
                CameraSubsystem.CameraDataProvider.OnFramesReady -= AdvanceFrame;
                ReleaseDepthFrames();
                session = null;
                m_HasBoundSession = false;
            }

            public override void Destroy() { }

            // ----------------------------------------------------------------
            // GPU texture access (called by AROcclusionManager each frame)
            // ----------------------------------------------------------------
            public override bool TryGetEnvironmentDepth(out XRTextureDescriptor environmentDepthDescriptor)
            {
                EnsureSessionInitialized();
                return TryBuildCurrentDescriptor(out environmentDepthDescriptor);
            }

            public override NativeArray<XRTextureDescriptor> GetTextureDescriptors(
                XRTextureDescriptor defaultDescriptor, Allocator allocator)
            {
                EnsureSessionInitialized();

                if (!TryBuildCurrentDescriptor(out var descriptor))
                    return new NativeArray<XRTextureDescriptor>(0, allocator);

                var descriptors = new NativeArray<XRTextureDescriptor>(1, allocator);
                descriptors[0] = descriptor;
                return descriptors;
            }

            public override XRResultStatus TryGetFrame(Allocator allocator, out XROcclusionFrame frame)
            {
                EnsureSessionInitialized();

                if (!framesReady || m_CurrentDepthTexture == null)
                {
                    frame = default;
                    return new XRResultStatus(XRResultStatus.StatusCode.ProviderNotStarted);
                }

                ARSensorFlexSession.GetPreferredClipPlanes(out float nearClipPlane, out float farClipPlane);

                var poses = new NativeArray<Pose>(1, allocator);
                poses[0] = PoseBridge.HasPose ? PoseBridge.LatestPose : Pose.identity;

                var textureDims = SfzSessionStore.LatestTextureDimensions;
                var fovs = new NativeArray<XRFov>(1, allocator);
                fovs[0] = BuildFov(SfzSessionStore.LatestIntrinsics, textureDims);

                frame = new XROcclusionFrame(
                    XROcclusionFrameProperties.Timestamp |
                    XROcclusionFrameProperties.NearFarPlanes |
                    XROcclusionFrameProperties.Poses |
                    XROcclusionFrameProperties.Fovs,
                    SfzSessionStore.LatestTimestampNs,
                    new XRNearFarPlanes(nearClipPlane, farClipPlane),
                    poses,
                    fovs);

                return XRResultStatus.unqualifiedSuccess;
            }

            // ----------------------------------------------------------------
            // Helpers
            // ----------------------------------------------------------------
            void EnsureSessionInitialized()
            {
                if (m_HasBoundSession)
                {
                    if (m_IsSfzMode)
                        EnsureSfzLoaderReady();
                    return;
                }

                session = ARSensorFlexSession.ResolveActiveSession();
                if (session == null)
                {
                    if (!m_LoggedWaitingForSession)
                    {
                        Debug.Log("[SF] OcclusionSubsystem: waiting for ARSensorFlexSession to become available.");
                        m_LoggedWaitingForSession = true;
                    }

                    return;
                }

                m_HasBoundSession = true;
                m_LoggedWaitingForSession = false;
                m_CurrentDepthWorldScale = session.EffectiveDepthWorldScale;

                if (!session.DepthEnabled)
                {
                    Debug.Log("[SF] OcclusionSubsystem: depth disabled on the active session.");
                    return;
                }

                if (session.SourceMode == ARSensorFlexSession.FrameSourceMode.Sfz ||
                    session.SourceMode == ARSensorFlexSession.FrameSourceMode.FileIo)
                {
                    m_IsSfzMode = true;
                    Debug.Log($"[SF] OcclusionSubsystem: {session.SourceMode} mode — reading depth from ring buffer.");
                    return;
                }

                Debug.LogWarning($"[SF] OcclusionSubsystem: unhandled source mode {session.SourceMode} — depth disabled.");
            }

            void ReleaseDepthFrames()
            {
                if (m_SfzDepthTexture != null)
                {
                    UnityEngine.Object.Destroy(m_SfzDepthTexture);
                    m_SfzDepthTexture = null;
                }

                m_CurrentDepthTexture = null;
                framesReady = false;
                m_IsSfzMode = false;
                m_LastSfzPlayHead = -1;
            }

            bool TryBuildCurrentDescriptor(out XRTextureDescriptor descriptor)
            {
                if (!framesReady || m_CurrentDepthTexture == null)
                {
                    descriptor = default;
                    return false;
                }

                var depthFmt = m_CurrentDepthTexture is Texture2D dt2d ? dt2d.format : TextureFormat.RFloat;
                descriptor = new XRTextureDescriptor(
                    m_CurrentDepthTexture.GetNativeTexturePtr(),
                    m_CurrentDepthTexture.width,
                    m_CurrentDepthTexture.height,
                    m_CurrentDepthTexture.mipmapCount,
                    depthFmt,
                    EnvironmentDepthPropertyId,
                    0,
                    XRTextureType.Texture2D
                );

                return true;
            }

            static XRFov BuildFov(Vector4 intrinsics, Vector2Int textureDimensions)
            {
                float fx = intrinsics.x;
                float fy = intrinsics.y;
                float cx = intrinsics.z;
                float cy = intrinsics.w;
                int width = Mathf.Max(1, textureDimensions.x);
                int height = Mathf.Max(1, textureDimensions.y);

                if (fx <= 0f || fy <= 0f)
                {
                    const float fallbackHalfFov = 30f * Mathf.Deg2Rad;
                    return new XRFov(-fallbackHalfFov, fallbackHalfFov, fallbackHalfFov, -fallbackHalfFov);
                }

                float left = -Mathf.Atan(cx / fx);
                float right = Mathf.Atan((width - cx) / fx);
                float up = Mathf.Atan(cy / fy);
                float down = -Mathf.Atan((height - cy) / fy);
                return new XRFov(left, right, up, down);
            }

            void AdvanceFrame()
            {
                if (m_IsSfzMode)
                    UpdateSfzDepthTexture();
            }

            void EnsureSfzLoaderReady()
            {
                if (framesReady)
                    return;

                if (!SfzSessionStore.IsReady)
                    return;

                // SFZ: the session store decodes depth.bin into an RFloat texture (metres),
                // updated by the Camera subsystem when a new RGB frame fires.
                if (SfzSessionStore.V2DepthTexture != null)
                {
                    framesReady = true;
                    Debug.Log("[SF] OcclusionSubsystem: SFZ depth texture ready.");
                    return;
                }

                // SFZ session without playable depth (none recorded, or an unsupported encoding).
                if (SfzSessionStore.IsVideoMode)
                    return;

                m_SfzDepthTexture = new Texture2D(RawDepthWidth, RawDepthHeight, TextureFormat.RFloat, false);
                framesReady = true;
                Debug.Log("[SF] OcclusionSubsystem: ZIP depth texture allocated.");
            }

            void UpdateSfzDepthTexture()
            {
                EnsureSfzLoaderReady();
                if (!framesReady)
                    return;

                // SFZ: Camera subsystem already decoded depth via UpdateVideoDepthForFrame.
                // Just point to the current texture — no further work needed here.
                var v2Depth = SfzSessionStore.V2DepthTexture;
                if (v2Depth != null)
                {
                    m_CurrentDepthTexture = v2Depth;
                    return;
                }

                if (m_SfzDepthTexture == null)
                    return;

                if (SfzSessionStore.DepthBins == null)
                    return;

                int playHead = SfzSessionStore.PlayHead;
                if (playHead < 0 || playHead == m_LastSfzPlayHead)
                    return;

                int slot = playHead % SfzSessionStore.BufSize;

                if (SfzSessionStore.SlotReady == null || !SfzSessionStore.SlotReady[slot])
                    return;
                if (SfzSessionStore.SlotGlobalIdx == null || SfzSessionStore.SlotGlobalIdx[slot] != playHead)
                    return;

                byte[] depthBytes = SfzSessionStore.DepthBins[slot];
                if (depthBytes == null)
                {
                    m_CurrentDepthTexture = null;
                    m_LastSfzPlayHead = playHead;
                    return;
                }

                int expectedBytes = RawDepthWidth * RawDepthHeight * sizeof(float);
                if (depthBytes.Length != expectedBytes)
                {
                    Debug.LogWarning(
                        $"[SF] OcclusionSubsystem: ZIP depth slot {slot} wrong size." +
                        $" expected={expectedBytes} actual={depthBytes.Length}");
                    m_CurrentDepthTexture = null;
                    m_LastSfzPlayHead = playHead;
                    return;
                }

                // Fast path: little-endian platform, no world scale — blit raw bytes directly.
                if (BitConverter.IsLittleEndian && Mathf.Approximately(m_CurrentDepthWorldScale, 1f))
                {
                    m_SfzDepthTexture.SetPixelData(depthBytes, 0);
                }
                else
                {
                    var floats = new float[RawDepthWidth * RawDepthHeight];
                    Buffer.BlockCopy(depthBytes, 0, floats, 0, depthBytes.Length);

                    if (!BitConverter.IsLittleEndian)
                    {
                        for (int i = 0; i < floats.Length; i++)
                        {
                            var b = BitConverter.GetBytes(floats[i]);
                            Array.Reverse(b);
                            floats[i] = BitConverter.ToSingle(b, 0);
                        }
                    }

                    if (!Mathf.Approximately(m_CurrentDepthWorldScale, 1f))
                    {
                        for (int i = 0; i < floats.Length; i++)
                            if (floats[i] > 0f)
                                floats[i] *= m_CurrentDepthWorldScale;
                    }

                    m_SfzDepthTexture.SetPixelData(floats, 0);
                }

                m_SfzDepthTexture.Apply(false);
                m_CurrentDepthTexture = m_SfzDepthTexture;
                m_LastSfzPlayHead = playHead;
            }
        }
    }
}
