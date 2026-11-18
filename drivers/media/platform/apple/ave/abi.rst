.. SPDX-License-Identifier: GPL-2.0-only

Apple AVE firmware ABIs
======================

The firmware TEXT CRC chooses the wire layout. SoC match data supplies register
addresses, boot arguments and device identity. These are separate from streaming
qualification: identifying a firmware does not qualify its encoder payloads.

9003.78.0 keeps its existing boot request, host clock synchronization, command
IDs, contiguous surface descriptors and reply envelope. The KUnit oracle in
``tests/ave-9003-reference.h`` freezes the driver builders and layout from Linux
commit 23e431a98962. Tests compare every byte of AVC and HEVC Start and Process
at 1920x1080 with B-frames enabled/disabled, and 3840x2160 with B-frames enabled.
They compare both deterministic synthetic DART addresses and masked images.
The constant image tables in ave-image.c are unchanged by this refactor.

6070.11.1 is selected for T6000 TEXT CRC 460c7a7c, T8112 TEXT CRC 25bfd039,
or T6021 TEXT CRC 0d56db87 (AppleAVE2FW_H14C, TEXT 0xf0000, DATA 0x134000),
with the corresponding SoC identity. Boot, seven-channel discovery, Config,
Open, Close and scratch-confirmed Halt are implemented from the clean derived
SPEC-6070-lifecycle.md ledger. Start and ordinary Process writers implement the
clean revision-2/P1 field ledgers: fixed-capacity address banks, split output
arrays, four interleaved entropy sets, rational frame rate, one initial-QP word,
retained codec-state partitions and per-slot HEVC slice-header tables. The
session allocator provides distinct backing for every live bank and rejects
non-16-KiB-aligned IOVAs.

Streaming is enabled for one 8-bit NV12 session of H.264 (Baseline, Main or
High) or HEVC Main, frame widths in whole macroblocks. Constant quality is the
default. CBR and VBR are firmware rate-control ON; CBR padding is done on the
host, and a bitrate written while streaming is read again on the next Process.
B-frames default to none, so each frame is answered before the next is
submitted. The firmware's parameter sets omit VUI timing. When a VUI is
already present the driver sets the timing flag in the output SPS. When
B-frames are off and the VUI flag itself is clear, the driver inserts a
minimal VUI (H.264 time_scale is twice the frame rate; HEVC time_scale is
the frame rate, and sps_extension_present_flag stays clear). Main 10, speed
priority, standalone LRME and Reset remain unavailable. No Priority command
is sent. Complete is
the normal drain barrier; Flush is reserved for cancellation before Stop and
Close.
KUnit checks the 1080p and 4K fields directly from the clean ledgers and embeds
no Apple command image.

Lifecycle-only validation
-------------------------

Build with CONFIG_DEBUG_FS. On a supported 6070 device, write ``1`` (AVC) or ``2``
(HEVC) to ``/sys/kernel/debug/apple-ave-<device>/lifecycle_test``. Each encoder
has its own directory, so a second instance is not dropped. On t6021 those
directories are ``apple-ave-40d100000.video-codec`` and
``apple-ave-507100000.video-codec``. This synchronous operation
runs boot, Config, Open, Close and Halt, then requests runtime suspend. It does
not allocate encoding surfaces or issue Start, Process or uninit for an encoder
that was never initialized. A second write exercises a cold restart and DATA
restore. An existing session makes the operation return EBUSY.

The platform must supply the extra-0 register resource as ``extra0`` or register
resource index 3, with at least 0x24 bytes on T6000, 0x4000 bytes on T8112, or
0x3dfc bytes on T6021 (the smaller power-control window). The driver uses the
translated platform resource address. The DMA API maps the pages covering that
range, including a base that is not 16 KiB aligned, and Config receives that
IOVA. Platform DT/binding changes are separate from this driver change.
No fixed firmware or register IOVA is used as an ordinary DMA allocation.

Before every boot the driver checks the firmware's IOMMU translations. DATA must
match the loader-exported pre-start CRC at probe; subsequent cold boots restore
that pristine image and verify its CRC. Firmware reservations and translations
remain persistent. IPC and extra-0 mappings are per-boot resources.

Halt clears scratch0, sends ID 14 without expecting a ring reply, and waits up to
five seconds for scratch0 0x08042006. Only then does the driver stop the IOP and
check control readback. On 6070 a confirmed Stop and Close do not Halt: the CPU
domain stays powered under the DARTs, and a later boot gets no hello. The
firmware stays up and the next session is Open and Start only. One runtime PM
reference taken after the first successful boot keeps autosuspend from Halting
it. System sleep still Halts. A Halted 6070 core (sleep, or a Close the firmware
did not confirm) is not booted again in a loop: two attempts, then -EIO, and
further STREAMON on that open fd returns without touching the core. The flag
clears when the last fd closes. A missing Halt confirmation or failed lifecycle
leaves the firmware broken and retains DMA mappings, runtime power and the
module reference. A forced unload or in-place restart is not a recovery procedure.

Hardware acceptance remains independent of KUnit: check both SoC VCBAR readbacks,
exact hello, ring dimensions, Config/Open/Close completion IDs and status, Halt
scratch confirmation, actual power gating and a second cold boot. Encoding on
9003 requires the existing hardware regression lane; these software tests do not
establish hardware equivalence.

The 9003 runtime recovery path retains its upstream quarantine and reset-witness
checks. It is deliberately unavailable for 6070: a broken lifecycle must keep
its mappings and power until scratch-confirmed Halt can establish quiescence.
