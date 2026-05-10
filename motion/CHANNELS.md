# miniOS motion — channels & cross-channel synchronisation

## What a channel is

A **channel** is an independent motion "machine": its own axis group, its
own program stream (G-code / G-code-like / cam-plan), its own
look-ahead queue, its own interpreter FSA, its own feedhold / overrides.
On a mill-turn you typically have two channels:

- **Channel 1 — mill side**: C1 (main spindle rotary), X, Y, Z, B
  (tool rotary). Runs the mill program.
- **Channel 2 — lathe side**: C2 (sub-spindle rotary), B2, Z2, X2.
  Runs a separate turning / parting / bar-feed program.

Channels are close to *separate machines that share the EtherCAT bus and
real-time kernel*. They must be able to run completely independent
programs simultaneously and must not stall each other except at
explicit sync points.

## Why two channels are not two unrelated machines

Two things force them to share kernel state:

1. **One EtherCAT master, one cycle.** There is a single LRW frame per
   cycle; all axes on both channels are sampled/commanded on the same
   SYNC0 edge. This is already true because DC is bus-wide; it is the
   foundation every sync primitive below leans on.
2. **Rendezvous at sub-count precision.** Operations like part transfer
   between spindles, synchronous parting, or electronic gearing require
   positional agreement between axes on different channels tighter than
   a single encoder count — user stated **0.1 µm / 0.0001°**. That is
   below the per-move following-error of any commercial servo, so it
   cannot be achieved as "both channels get there eventually"; it has
   to be an explicit kernel-enforced barrier with a tolerance check.

## The three sync primitives

Everything a mill-turn program needs composes from three primitives.
These are all *kernel* objects, not just interpreter directives —
because they have to be honoured cycle-by-cycle in hard real time.

### 1. Barrier / rendezvous (G10.1-style, "wait for channel N")

Each channel can emit a named `barrier_token` in its program stream.
The kernel tracks arrivals. A channel that hits a barrier stops its
look-ahead consumption and holds its current commanded position.
When all participants are present *and* each participant's
`|commanded - actual| < tolerance` for ≥ `stable_cycles` consecutive
cycles, the barrier releases and all channels resume on the same
cycle edge.

- Tolerance is per-axis, not per-channel, and configurable per barrier
  (`G10.1_POS_TOL=0.0001` etc.). Default per axis class: 0.1 µm linear,
  0.0001° rotary.
- `stable_cycles` defaults to 3 to ride out encoder noise and the
  one-cycle of dead-time between commanded and actual.
- If a participant never converges (tolerance never met), the barrier
  expires after `max_wait_cycles` and raises `SyncTimeout` as a motion
  fault — same path as any other axis fault.

### 2. Electronic gearing / phase-lock (threading, rigid-tap, hobbing)

A follower axis tracks a leader axis at a fixed ratio:
`follower.commanded[t] = base + k · (leader.actual[t] - leader_base)`.

- Must be computed and commanded *in the same cycle* the leader sample
  arrives — latency between leader-sample and follower-command is what
  ruins a thread.
- Leader and follower may live on different channels (e.g. C1 main
  spindle as leader, Z on channel 2 as follower for a cross-channel
  threading operation).
- Ratio `k` is allowed to be non-integer and signed; phase `base` is
  captured at engagement.
- Engagement has a ramp: over `N` cycles blend from `k_old=0` to
  `k_new`, else the follower yanks. Disengagement mirrors this.
- While geared, the follower's normal look-ahead is frozen — the
  follower's program is not allowed to command against the gearing.
  Attempts to do so are an interpreter error, not a runtime fault.

### 3. Synchronous multi-axis move across channels

Sometimes you need channel 1 and channel 2 to both arrive at specified
positions in the same cycle — part handoff from the main to the sub
spindle while both are still rotating, for example.

This is a generalisation of (1): a *coordinated move* where both
channels plan a trajectory whose `T_final` is identical and whose
per-cycle commanded positions are shared to one look-ahead scheduler.
The look-ahead must pick whichever channel's profile is slowest and
scale the faster channel to match — identical to multi-axis coordinated
moves inside one channel, just crossing the channel boundary.

## Non-primitives (things that fall out for free)

- **Feedhold per channel.** Kernel already supports per-axis stop; a
  channel-wide hold is the intersection of per-axis stops for the
  channel's axes. No new mechanism.
- **Override per channel.** Feed/rapid/spindle overrides apply to a
  channel's axes; other channels are unaffected.
- **Independent fault domains.** A following-error on channel 2's X2
  must fault channel 2, not channel 1 — unless the two are in a
  barrier or gearing relationship, in which case the fault propagates
  to the entangled channel. Propagation is explicit, not implicit.

## How this maps onto the existing motion kernel

Today `motion::` has a flat axis array and a single look-ahead FSA.
What changes:

1. **New `motion::Channel` struct** owning:
   - `axes`: index list into the global `Axis[]` pool (not owning — the
     EtherCAT master still owns the drive handles).
   - `program_source`: interpreter / cam-plan iterator.
   - `lookahead`: ring of planned segments, per-channel.
   - `fsa`: Idle / Running / FeedHold / WaitingBarrier / Gearing / Fault.
   - `overrides`: feed/rapid/spindle scalars.
2. **Global `motion::Kernel`** owns channels[] and the three sync
   primitives' shared state (barrier tokens, gearing relationships).
3. **Per-cycle tick** iterates channels in a fixed order, advancing
   each channel's FSA, then runs the sync arbiter which can block or
   release channels, then commits the cycle's outputs to EtherCAT.
4. **Axis-to-channel binding** happens at config time, from a TSV or
   equivalent — e.g.
   `channel=1 axes=X,Y,Z,C1,B` / `channel=2 axes=X2,Z2,C2,B2`. An axis
   belongs to exactly one channel (never two) to keep ownership
   uncontested; cross-channel relationships go through sync primitives.

## Kernel-level correctness invariants

1. **Single cycle, all channels.** Every axis command going to the
   EtherCAT LRW frame in cycle N was computed from state valid in
   cycle N-1 — no channel is ever one cycle behind another. Anything
   else makes 0.0001° sync impossible.
2. **Ownership is exclusive.** Exactly one channel commands any given
   axis. A gearing relationship *reads* a leader's actual but does not
   steal command authority.
3. **Barrier atomicity.** Release of a barrier happens to all
   participants in the same cycle — never partial. If the arbiter
   releases channel 1 in cycle N and channel 2 in cycle N+1, threading
   and part-handoff are already broken.
4. **Fault propagation is explicit.** A fault crosses channel lines
   only through a sync primitive that has explicitly declared
   propagation. This avoids cascading stops during unrelated work.

## What to prototype first (tied to PLAN.md Phase 9)

1. Split current flat motion kernel into `Channel` + global `Kernel`
   with a single channel; prove the rewrite is behaviourally
   equivalent before adding a second channel.
2. Second channel with its own interpreter iterator, independent
   tick — run two trivially-different programs and confirm neither
   stalls the other.
3. Barrier primitive with tolerance + stable-cycle check. Validate
   via a test program where channel 1 and channel 2 try to meet at a
   named mark with arbitrary pre-travel; assert both axes are within
   tolerance for the required number of cycles before release.
4. Electronic gearing between leader axis on channel 1 and follower
   on channel 2. Threading test: command spindle C1 to spin, engage
   Z2 at a known ratio, measure Z2 position vs C1 position over 1000
   cycles. Phase drift must be bounded.

## Mill-turn G-code conventions (Phase B)

Once the kernel has channels + barriers + gearing, programs need a
small surface of M-codes to drive them. Fixed convention so a part
program written against this kernel is portable across machines:

### Sync barriers — M100 / M110 / M200 / M101..M109

Same primitive (`Kernel::arrive_at_barrier`); ten distinct M-codes so
a program can have multiple sync points without colliding tokens.

- `M100 P<token> Q<channels-mask>` — generic rendezvous. Both `P`
  and `Q` are optional. When `P` is omitted the token defaults to
  `(mcode * 1000 + line) & 0xFFFF` so two `M100` lines on different
  source lines automatically don't collide. When `Q` is omitted the
  participants mask defaults to `0x3` (channel 0 + channel 1).
- `M110`, `M200` — historical aliases of `M100`. Same shape.
- `M101..M109` — named alternates. Useful when the source program
  has a logical "after roughing", "after finishing", "after
  part-off" sequence and the operator wants those distinct in
  `barriers` / `sync_status` output.
- Tolerance / stable_cycles / max_wait are kernel defaults today
  (1 cnt / 3 cycles / 20 000 cycles ≈ 5 s at 250 µs). Per-program
  tightening lands when a `G10.1 L<tol>` style verb is added.

### Override enable/disable — M48 / M49

- `M48` — feed and spindle override sliders are honoured for this
  channel. Default at program start.
- `M49` — overrides are pinned at 100 %. Used for synchronised cuts
  (threading, rigid tap) where the operator's slider would otherwise
  desynchronise the leader/follower phase relationship.

The interpreter scales `feed_cps` and the spindle command by the
channel's `feed_permille` / `spindle_permille` whenever
`override_active` is true. M48/M49 toggle that flag.

### Fault propagation across barriers (fault-fast-release)

If any participant on an active barrier transitions to `Fault`
(following error, drive trip, hard limit, ...) the arbiter
immediately moves every other waiting participant to `Fault` in the
same cycle. Without this fast-release, a peer fault left siblings
stuck at `WaitingBarrier` for the full `max_wait_cycles` budget; the
operator now sees the trip on every channel within one tick.

### sync_status CLI

`sync_status` is the one-shot mill-turn dashboard:

```
ch0(mill) state=running feed=100% rapid=100% spindle=100% jd=10
  prog=part.ngc line=42 block=39 interp=running feed=600 spindle=2400 G43
ch1(lathe) state=barrier feed=100% rapid=100% spindle=100% jd=10
  prog=lathe.ngc line=78 block=66 interp=barrier feed=400 spindle=0
  waiting on token=0x004f mask=0x03
--- barriers ---
[slot 0] token=0x004f parts=03 arrived=02 ...
```

Use this instead of stitching together `channels`, `barriers`, and
`program_status` when triaging a stuck mill-turn program.
