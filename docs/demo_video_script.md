# SORT — Demo Video Script (target 2:30)

Narration budget ≈ 150 wpm. Timecodes are cumulative.
The study was a gesture **elicitation**: participants were asked, unprompted, how the arm
should move after a correct, false, or uncertain sort. Only 9, the participant count,
still needs filling in.

---

## 0:00 – 0:18 · Showing the setup

**VISUAL:** Static wide shot of RoboLab corner with our table and setup. Kinova Gen3 hanging from the ceiling, table
below, three DinA4 AprilTag zones, four tagged cubes beside them. A hand enters, places
cube 1 into zone 1. The arm nods. Cut to title card: **SORT — Separation-Oriented
Recognition & Tagging**.

**VO:** "When a robot watches you work, how does it tell you that you got it right —
without a screen, and without saying a word?"

---

## 0:18 – 0:42 · The task: not showing the full thing yet, only exactly what zones and cubes there are

**VISUAL:** Top-down camera view with zone overlays drawn on. Label each cube as it is
pointed at: cube 1 → zone 1, cube 2 → zone 2, cube 3 → zone 3, cube 4 → *no zone*.

**VO:** "SORT supervises a zone-based sorting task. Three zones, four cubes, all tracked
by AprilTags. Cubes one to three each belong to one zone. Cube four belongs to none, it is
our deliberate ambiguity (uncertain feedback). The robot watches, judges every placement, and answers in three
ways: correct, incorrect, or uncertain."

---

## 0:42 – 1:00 · Why gesture

**VISUAL:** Split screen: left, the light strip turning green; centre, the arm nodding;
right, a waveform for the TTS line.

**VO:** "Feedback runs over three channels at once — a Home Assistant light, a spoken
line, and the arm's own movement. The light and the voice were easy. The movement was not:
the arm has no gripper and no face, so every signal has to live in six joints."

---

## 1:00 – 1:45 · The gesture design space + study

**VISUAL:** Cut between three participant clips — a person gesturing with their own head
and hands as they answer — and the matching arm gesture we built from it. Three pairs,
~7 s each, in the order correct → false → uncertain.

**VO:** "We asked 9 people 3 questions. "After you sort something, how should the arm move to tell you the result?" When it is sorted correctly, falsely or when the robot arm is uncertain. After you sort something, how should the arm move to tell you the result?

The answers converged, and they converged on the human head. For a correct placement,
everyone described a nodding motion. For a false one, everyone reached for a head shake.
And for uncertainty, the case we expected to be hardest, the majority expected people tilted their heads.

So we did not have to choose a vocabulary. We had to translate one, into an arm that has no head and no face: six joints, hanging from the ceiling."

**On-screen table (freeze frame):**

| State | Elicited gesture | How it should be |
|---|---|---|
| Correct | Nod | relatively fast, horizontal motion |
| False | Head shake | relatively fast vertical motion |
| Uncertain | Head tilt | slower than correct or false, tilting in an angle and rotating on a vertical plane|

**VO tag (over the table):** "The tilt taught us the one principle we kept. "Uncertainty is when the arm stops looking decisive and starts
looking unsure."

---

## 1:45 – 2:20 · The system, live — all three states

**VISUAL:** Three uncut takes, ~11 s each. Picture-in-picture corner shows the terminal
log line from `feedback_node` so the state name and reason are visible on screen.

**Take A — CORRECT.** Hand places cube 2 in zone 2.
→ light green, arm nods, voice: *"You sorted it correctly. Well done!"*
→ log: `Sorting state: CORRECT (all 4 visible cube(s) correct)`

**Take B — INCORRECT.** Hand places cube 3 in zone 1.
→ light red, arm shakes, voice: *"You sorted it wrong. You can do better!"*
→ log: `Sorting state: INCORRECT (1 cube(s) in the wrong zone)`

**Take C — UNCERTAIN.** Hand places cube 4 into a zone, then a second take where a hand
occludes a zone tag.
→ light yellow, arm tilts, voice: *"I am uncertain with your sorting. Please double check it."*
→ log: `Sorting state: UNKNOWN (no zone tags visible)`

**VO:** "Detection runs on AprilTags fused across two cameras into a shared world model.
Every second, the decision handler asks what that world means: are all visible cubes in
their target zones? Is anything in no zone at all? Can it even see the zones? The verdict
picks the channel triple — colour, sentence, gesture — and it only fires when the state
*changes*, so the arm is not nodding once a second at a table nobody is touching."

---

## 2:20 – 2:35 · Close

**VISUAL:** Slow push-in on the arm returning to home position. Text overlay of the two
open items.

**VO:** "Two things are still ahead of us. A gripper, so the arm can correct the mistake instead of only reporting it. And a second study that runs the other direction, because so far we have only asked people what they would *produce*. We have not yet checked whether people outside of this practical *read* it correctly coming back off the machine."

**END CARD:** SORT · RoboLab · `github.com/JHPNE/SORT`

---

## Pre-shoot checklist

- [ ] `export ROS_DOMAIN_ID=0` in *every* terminal, on both the RoboLab PC and the VM. -> Start the script:
```
cd ~/ros2_ws
chmod +x start_sort.sh
./start_sort.sh
```
- Light conditions good
- Robot, Lights, Tts check
