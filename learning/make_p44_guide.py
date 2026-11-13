"""Build learning/P4_4_ACT_guide.pdf: a learner's guide to P4.4 (ACT imitation learning)."""
import json
import sys
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.graphics.shapes import Drawing, Line, PolyLine, Rect, String, Circle, Polygon
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (KeepTogether, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table,
                                TableStyle, ListFlowable, ListItem)

ROOT = Path(sys.argv[1])
P4 = ROOT / "projects" / "p4_vision"
OUT = ROOT / "learning" / "P4_4_ACT_guide.pdf"

F = "C:/Windows/Fonts/"
pdfmetrics.registerFont(TTFont("Arial", F + "arial.ttf"))
pdfmetrics.registerFont(TTFont("Arial-Bold", F + "arialbd.ttf"))
pdfmetrics.registerFont(TTFont("Arial-Italic", F + "ariali.ttf"))
pdfmetrics.registerFont(TTFont("Consolas", F + "consola.ttf"))
from reportlab.lib.fonts import addMapping
addMapping("Arial", 0, 0, "Arial"); addMapping("Arial", 1, 0, "Arial-Bold"); addMapping("Arial", 0, 1, "Arial-Italic")

# palette (dataviz reference, light mode)
INK, INK2, MUTED = colors.HexColor("#0b0b0b"), colors.HexColor("#52514e"), colors.HexColor("#898781")
GRID, AXIS, SURF = colors.HexColor("#e1e0d9"), colors.HexColor("#c3c2b7"), colors.HexColor("#fcfcfb")
S1, S2, S3 = colors.HexColor("#2a78d6"), colors.HexColor("#eb6834"), colors.HexColor("#1baf7a")
BOXBG = colors.HexColor("#f3f2ee")

st = dict(
    h1=ParagraphStyle("h1", fontName="Arial-Bold", fontSize=20, leading=24, textColor=INK, spaceAfter=6),
    sub=ParagraphStyle("sub", fontName="Arial", fontSize=11, leading=15, textColor=INK2, spaceAfter=14),
    h2=ParagraphStyle("h2", fontName="Arial-Bold", fontSize=14, leading=18, textColor=INK, spaceBefore=14, spaceAfter=6),
    h3=ParagraphStyle("h3", fontName="Arial-Bold", fontSize=11, leading=14, textColor=INK, spaceBefore=8, spaceAfter=3),
    p=ParagraphStyle("p", fontName="Arial", fontSize=10, leading=14.2, textColor=INK, spaceAfter=6, alignment=TA_LEFT),
    cap=ParagraphStyle("cap", fontName="Arial-Italic", fontSize=8.8, leading=12, textColor=INK2, spaceBefore=3, spaceAfter=10),
    box=ParagraphStyle("box", fontName="Arial", fontSize=9.6, leading=13.5, textColor=INK),
    cell=ParagraphStyle("cell", fontName="Arial", fontSize=8.8, leading=11.5, textColor=INK),
    cellb=ParagraphStyle("cellb", fontName="Arial-Bold", fontSize=8.8, leading=11.5, textColor=INK),
    code=ParagraphStyle("code", fontName="Consolas", fontSize=8.4, leading=11, textColor=INK, backColor=BOXBG,
                        borderPadding=5, spaceBefore=4, spaceAfter=10),
)
W = A4[0] - 40 * mm


def P(t, s="p"):
    return Paragraph(t, st[s])


def bullets(items):
    return ListFlowable([ListItem(P(i), leftIndent=10, value="•") for i in items], bulletType="bullet",
                        start="•", leftIndent=12, bulletFontName="Arial", bulletFontSize=9)


def callout(title, body):
    t = Table([[P(f"<b>{title}</b><br/>{body}", "box")]], colWidths=[W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), BOXBG), ("LEFTPADDING", (0, 0), (-1, -1), 9),
                           ("RIGHTPADDING", (0, 0), (-1, -1), 9), ("TOPPADDING", (0, 0), (-1, -1), 7),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 7), ("LINEBEFORE", (0, 0), (0, -1), 2.5, S1)]))
    return t


def table(rows, widths, head=True):
    data = [[P(c, "cellb" if (head and i == 0) else "cell") for c in r] for i, r in enumerate(rows)]
    t = Table(data, colWidths=widths, repeatRows=1 if head else 0)
    t.setStyle(TableStyle([("LINEBELOW", (0, 0), (-1, 0), 0.8, AXIS), ("LINEBELOW", (0, 1), (-1, -1), 0.3, GRID),
                           ("VALIGN", (0, 0), (-1, -1), "TOP"), ("TOPPADDING", (0, 0), (-1, -1), 3),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]))
    return t


# ------------------------------------------------------------------ figures
def line_chart(series, xlab, ylab, xmax, ymax, yticks, xticks, w=W, h=62 * mm, labels=None, vlines=()):
    d = Drawing(w, h)
    L, B, R, T = 34, 26, 70, 18
    pw, ph = w - L - R, h - B - T
    X = lambda x: L + x / xmax * pw  # noqa: E731
    Y = lambda y: B + min(y, ymax) / ymax * ph  # noqa: E731
    for yt in yticks:
        d.add(Line(L, Y(yt), L + pw, Y(yt), strokeColor=GRID, strokeWidth=0.4))
        d.add(String(L - 4, Y(yt) - 3, f"{yt:g}", fontName="Arial", fontSize=7, fillColor=MUTED, textAnchor="end"))
    for xt in xticks:
        d.add(String(X(xt), B - 10, f"{xt:g}", fontName="Arial", fontSize=7, fillColor=MUTED, textAnchor="middle"))
    d.add(Line(L, B, L + pw, B, strokeColor=AXIS, strokeWidth=0.8))
    for xv, txt in vlines:
        d.add(Line(X(xv), B, X(xv), B + ph, strokeColor=AXIS, strokeWidth=0.6, strokeDashArray=[2, 2]))
        d.add(String(X(xv) + 3, B + ph - 8, txt, fontName="Arial", fontSize=7, fillColor=INK2))
    d.add(String(L + pw / 2, 3, xlab, fontName="Arial", fontSize=7.5, fillColor=INK2, textAnchor="middle"))
    d.add(String(4, h - 9, ylab, fontName="Arial", fontSize=7.5, fillColor=INK2))
    for (name, pts, col) in series:
        d.add(PolyLine([v for x, y in pts for v in (X(x), Y(y))], strokeColor=col, strokeWidth=1.6, strokeLineJoin=1))
        if len(pts) <= 12:
            for x, y in pts:
                d.add(Circle(X(x), Y(y), 2.2, fillColor=col, strokeColor=SURF, strokeWidth=0.8))
        x, y = pts[-1]
        d.add(String(X(x) + 6, Y(y) - 2.5, name, fontName="Arial", fontSize=7.5, fillColor=INK))
    return d


def bar_chart(rows, w=W, h=40 * mm):
    d = Drawing(w, h)
    L, R = 150, 40
    n = len(rows)
    bh, gap = 11, 6
    top = h - 6
    pw = w - L - R
    for k in (0, 5, 10, 15, 20, 25):
        x = L + k / 25 * pw
        d.add(Line(x, 12, x, top, strokeColor=GRID, strokeWidth=0.4))
        d.add(String(x, 2, str(k), fontName="Arial", fontSize=7, fillColor=MUTED, textAnchor="middle"))
    for i, (name, v, col) in enumerate(rows):
        y = top - (i + 1) * (bh + gap) + gap
        d.add(String(L - 6, y + 3, name, fontName="Arial", fontSize=8, fillColor=INK, textAnchor="end"))
        d.add(Rect(L, y, max(v / 25 * pw, 1.5), bh, fillColor=col, strokeColor=None, rx=2, ry=2))
        d.add(String(L + v / 25 * pw + 4, y + 3, f"{v}/25", fontName="Arial", fontSize=8, fillColor=INK))
    return d


def pipeline(w=W, h=58 * mm):
    d = Drawing(w, h)
    boxes = [("Expert grasp\n(classical, true pose)", 0), ("Demos v4\n250 episodes, DART", 1),
             ("LeRobot dataset\n117k frames, 3 cams", 2), ("ACT training\nColab L4, 20k steps", 3)]
    bw, bhh, gx = (w - 3 * 14) / 4, 34, 14
    y1 = h - bhh - 4
    for txt, i in boxes:
        x = i * (bw + gx)
        d.add(Rect(x, y1, bw, bhh, fillColor=BOXBG, strokeColor=AXIS, strokeWidth=0.6, rx=3, ry=3))
        a, b = txt.split("\n")
        d.add(String(x + bw / 2, y1 + 20, a, fontName="Arial-Bold", fontSize=7.6, fillColor=INK, textAnchor="middle"))
        d.add(String(x + bw / 2, y1 + 9, b, fontName="Arial", fontSize=7, fillColor=INK2, textAnchor="middle"))
        if i < 3:
            xa = x + bw
            d.add(Line(xa + 1, y1 + bhh / 2, xa + gx - 3, y1 + bhh / 2, strokeColor=INK2, strokeWidth=0.8))
            d.add(Polygon([xa + gx - 1, y1 + bhh / 2, xa + gx - 5, y1 + bhh / 2 + 2.5, xa + gx - 5, y1 + bhh / 2 - 2.5],
                          fillColor=INK2, strokeColor=None))
    # loop row
    y2 = 6
    row2 = [("Offline check\nval_act.py, 30 episodes", 3), ("Closed loop\neval_act.py, 25 seeds", 2),
            ("Diagnose\nshadow / hybrid", 1), ("Fix + retrain\ntime input, DAgger", 0)]
    for txt, i in row2:
        x = i * (bw + gx)
        fill = colors.HexColor("#fdf0ea") if i == 0 else BOXBG
        d.add(Rect(x, y2, bw, bhh, fillColor=fill, strokeColor=AXIS, strokeWidth=0.6, rx=3, ry=3))
        a, b = txt.split("\n")
        d.add(String(x + bw / 2, y2 + 20, a, fontName="Arial-Bold", fontSize=7.6, fillColor=INK, textAnchor="middle"))
        d.add(String(x + bw / 2, y2 + 9, b, fontName="Arial", fontSize=7, fillColor=INK2, textAnchor="middle"))
        if i > 0:
            xa = x
            d.add(Line(xa - 1, y2 + bhh / 2, xa - gx + 3, y2 + bhh / 2, strokeColor=INK2, strokeWidth=0.8))
            d.add(Polygon([xa - gx + 1, y2 + bhh / 2, xa - gx + 5, y2 + bhh / 2 + 2.5, xa - gx + 5, y2 + bhh / 2 - 2.5],
                          fillColor=INK2, strokeColor=None))
    xd = 3 * (bw + gx) + bw / 2
    d.add(Line(xd, y1 - 1, xd, y2 + bhh + 3, strokeColor=INK2, strokeWidth=0.8))
    d.add(Polygon([xd, y2 + bhh + 1, xd - 2.5, y2 + bhh + 5, xd + 2.5, y2 + bhh + 5], fillColor=INK2, strokeColor=None))
    d.add(String(w / 2, y1 - 12, "we are here: closed loop works poorly, diagnosed why, next is the fix (orange)",
                 fontName="Arial-Italic", fontSize=7.2, fillColor=INK2, textAnchor="middle"))
    return d


# data
steps = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]
off_all = [28.7, 22.2, 19.1, 15.9, 13.5, 12.2, 10.1, 9.7, 9.4, 8.7]
off_reach = [31.3, 26.9, 32.1, 30.2, 27.0, 22.3, 20.9, 20.0, 13.9, 18.2]
off_app = [24.4, 20.5, 12.2, 10.8, 9.2, 7.1, 7.0, 6.1, 7.1, 5.3]
fig_off = line_chart([("approach", list(zip(steps, off_app)), S1), ("reach", list(zip(steps, off_reach)), S2),
                      ("all phases", list(zip(steps, off_all)), S3)],
                     "training step (thousands)", "median position error (mm)", 21, 35, [0, 10, 20, 30],
                     [2, 4, 6, 8, 10, 12, 14, 16, 18, 20])

te = {e["seed"]: e for e in json.loads((P4 / "p44_val_act_act20k_te.json").read_text())["episodes"]}
sh = {e["seed"]: e for e in json.loads((P4 / "p44_val_act_shadow.json").read_text())["episodes"]}
seed = 6000001
lag_act = [(x[0], x[4]) for x in te[seed]["trace"]]
lag_exp = [(x[0], x[4]) for x in sh[seed]["trace"]]
fig_lag = line_chart([("ACT drives", lag_act, S2), ("expert drives", lag_exp, S1)],
                     "simulation time (s)", "gap: expert target to gripper (mm)", 56, 1000,
                     [0, 250, 500, 750, 1000], [0, 10, 20, 30, 40, 50], vlines=[(2, "reach"), (22, "approach")])

fig_bar = bar_chart([("Oracle (true pose, classical)", 25, S1), ("P4.3 estimator + classical", 25, S1),
                     ("ACT, temporal ensembling", 3, S2), ("ACT, chunk execution", 2, S2),
                     ("Hybrid: classical reach + ACT", 5, S2)])

# ------------------------------------------------------------------ document
story = []
story += [P("Learning to grasp from pixels: P4.4 explained", "h1"),
          P("mjorbit Project 4, step P4.4: imitation learning with ACT. A learner's guide to what we built, "
            "what the results mean, why the policy fails in closed loop, and where you can help. 13 Nov 2026.", "sub")]

story += [P("1. The big picture", "h2"),
          P("The end goal of mjorbit is <b>satellite servicing</b>: a servicer spacecraft with a robot arm flies up to a "
            "client satellite, grabs its <b>grapple bar</b>, and then repositions or reorients it. Projects 1 to 3 built "
            "the orbital simulator, the approach, and a <b>classical</b> grasp controller that works when it knows the "
            "exact pose of the bar. They also built the <b>E2 safety monitors</b>, which watch every episode and trigger "
            "a clean abort (back off, stow the arm) when something looks wrong."),
          P("Project 4 asks: <b>can the robot do this from its cameras?</b> It has three sensors: a wrist camera, a "
            "camera on the servicer bus, and a flash lidar (a depth image). Project 4 goes in two steps:"),
          bullets(["<b>P4.1 to P4.3 (done): perception only.</b> A neural network looks at the images and estimates the "
                   "bar pose (1.6 mm median error). The classical controller then uses that estimate. Result: "
                   "<b>25/25 grasps</b> on the realistic arm, same as with the true pose.",
                   "<b>P4.4 (now): learn the whole behaviour.</b> Instead of estimating a pose and handing it to "
                   "hand-written control, a policy network goes straight from images (plus the arm's own state) to "
                   "arm targets and the decision when to close the jaws. This is <b>imitation learning</b>: it learns "
                   "by copying the classical expert."]),
          callout("Why bother, if P4.3 already works?",
                  "Learned end-to-end policies are where modern robot manipulation is going (ACT, Diffusion Policy, "
                  "VLA models). They can, in principle, handle things a hand-written controller cannot: unknown client "
                  "shapes, occlusions, changing grasp strategies. P4.4 is the honest test of whether that works here, "
                  "with the classical system as a baseline and the E2 monitors as a safety shield.")]

story += [P("2. The pipeline we built", "h2"), pipeline(),
          P("Figure 1. The data and training pipeline (top row) and the evaluation and fix loop (bottom row).", "cap"),
          P("2.1 Demonstrations", "h3"),
          P("<font face='Consolas'>gen_demos.py</font> runs the classical expert in the simulator and records every 0.1 s "
            "(10 Hz): the three camera images, a 29-number <b>state</b> (arm joint angles and speeds, jaw positions, "
            "gripper pose and velocity in the bus frame) and the <b>action</b>: the expert's gripper target (position, "
            "orientation) plus a close flag. One episode is about 47 s: settle, reach (20 s), approach (about 17 s), "
            "close, hold."),
          P("We recorded 125 clean episodes and 125 with <b>DART noise</b>: a small random wobble (2 mm, 0.3 degrees) is "
            "added to what the arm actually does, but the label stays the clean expert target. This shows the network "
            "slightly-off states together with the correct way back, which is cheap insurance against drifting."),
          P("2.2 LeRobot dataset and ACT", "h3"),
          P("<font face='Consolas'>to_lerobot.py</font> converts the episodes into a <b>LeRobot</b> dataset (Hugging "
            "Face's robot-learning library): images become compressed videos (128 x 128), the lidar is packed as a grey "
            "image at 5.9 mm per grey level. 220 episodes train, 30 are held out for validation."),
          P("<b>ACT</b> (Action Chunking with Transformers, Zhao et al. 2023) is the policy. Per frame it takes the three "
            "images (through a ResNet-18 each) and the state, and a transformer predicts a <b>chunk</b>: the next 20 "
            "actions (2 s). Three things to know:"),
          bullets(["<b>Action chunking</b>: predicting a short plan instead of one step reduces how often small errors "
                   "can compound, and copes with demonstrations that pause.",
                   "<b>Temporal ensembling</b>: if you predict a new chunk every frame, each future frame is covered by "
                   "several chunks; averaging them smooths the motion. The alternative is to execute a chunk and only "
                   "re-plan every k frames.",
                   "ACT also has a small variational encoder (CVAE) for multi-modal demos. Ours are single-mode, and "
                   "the training log shows it unused (KL loss 0). That is harmless."]),
          P("Training ran on a Colab L4 GPU (the laptop overheats on long GPU jobs): 20,000 steps, batch 32, about 37 min, "
            "checkpoints every 2,000 steps. A resume to 40,000 steps was started."),
          ]

story += [P("3. Result 1: offline, it looks good", "h2"),
          P("<font face='Consolas'>val_act.py</font> feeds the 30 held-out episodes to each checkpoint and compares the "
            "predicted target with the expert's label, frame by frame. This is <b>open-loop</b> evaluation: the network "
            "always sees states that the expert produced."),
          fig_off,
          P("Figure 2. Offline validation: median gripper-target error per checkpoint. Approach, the phase that matters "
            "most, falls to 5.3 mm. The close flag is 100% correct.", "cap"),
          table([["Phase (20k checkpoint)", "Median / p95 position", "Median rotation"],
                 ["all", "8.7 / 33.5 mm", "0.70 deg"], ["reach", "18.2 / 43.0 mm", "2.11 deg"],
                 ["approach", "5.3 / 16.8 mm", "0.20 deg"], ["close", "3.1 / 6.8 mm", "0.17 deg"],
                 ["hold", "3.6 / 11.4 mm", "0.16 deg"]], [W * 0.4, W * 0.35, W * 0.25]),
          Spacer(1, 6)]

story += [PageBreak(), P("4. Result 2: in closed loop, it fails", "h2"),
          P("<font face='Consolas'>eval_act.py</font> lets ACT actually drive the arm in the simulator on 25 validation "
            "episodes, the same seeds used for the oracle and P4.3. The wrapper "
            "(<font face='Consolas'>act_policy.py</font>) renders the cameras every 0.1 s, asks ACT for a chunk, "
            "interpolates between the 10 Hz targets for the 100 Hz controller, and converts the bus-frame targets to "
            "world coordinates. Success means: captured, no abort, no collision, client pushed less than 2 mm/s."),
          fig_bar,
          P("Figure 3. Closed-loop successes out of 25 validation episodes. There were no collisions in any run: the E2 "
            "monitors aborted every bad episode cleanly.", "cap"),
          table([["Run", "What happened"],
                 ["ACT, temporal ensembling", "17 timed out with the arm stuck near its stowed pose; 5 safety aborts."],
                 ["ACT, chunk execution (re-plan every 20 frames)", "8 captured but 18 episodes pushed the client too hard; 10 safety aborts."],
                 ["Hybrid: classical reach, ACT approach and close",
                  "9 captured (4 too hard); 12 aborts, mostly jaws pushing the bar; ACT closed 19 times but only 5 of "
                  "those were inside the expert's 1 cm window."]], [W * 0.38, W * 0.62]),
          Spacer(1, 6)]

story += [P("5. Why: the policy has no clock", "h2"),
          P("First we ruled out a bug. In a <b>shadow run</b> the expert drives the arm while ACT predicts live from the "
            "real rendered frames. ACT's error there is 13 to 16 mm in reach and about 5 mm in approach, the same as "
            "offline. So the images, frames and normalisation are right; the problem appears only when ACT's own "
            "outputs move the arm."),
          fig_lag,
          P("Figure 4. One episode (seed 6000001): the distance between where the expert would want the gripper and where "
            "it actually is. When the expert drives (blue) it stays under a few cm. When ACT drives (orange) the arm "
            "falls behind and the gap grows to almost a metre: the arm barely leaves its stowed pose.", "cap"),
          P("The cause has two parts that feed each other:"),
          bullets(["<b>The expert follows a clock.</b> The reach is a smooth joint path that takes exactly 20 s; the "
                   "approach moves at exactly 1 cm/s. The right target depends on <i>how long the phase has been "
                   "running</i>, which is not in ACT's input.",
                   "<b>Copycat shortcut.</b> In the demos the gripper always sits just behind the target, so the easiest "
                   "thing to learn is \"target = where the gripper is, plus a little\". ACT's error (13 to 20 mm) is larger "
                   "than the expert's lead (5 to 7 mm), so once the arm falls behind, the predicted target is behind "
                   "too, and the arm never catches up.",
                   "This is the classic <b>covariate shift</b> of behaviour cloning (Ross et al. 2011): the policy is "
                   "trained only on states the expert visits, and its own small errors take it to states it never saw."]),
          callout("The key lesson",
                  "Good open-loop (offline) error does not mean good closed-loop behaviour. A policy is judged by "
                  "rolling it out, because its errors change what it sees next. Always check both.")]

story += [P("6. What comes next", "h2"),
          table([["Step", "Idea", "Cost"],
                 ["Retrain with time", "Add time-in-phase (or phase progress) to the state, so ACT knows where on the "
                  "expert's schedule it should be. Consider removing the gripper pose and velocity from the state to "
                  "break the copycat shortcut.", "Re-convert the dataset (about 16 min), new Colab run (about 40 min)"],
                 ["DAgger (P4.5)", "Run the policy, let the expert label the states the policy actually reaches, add "
                  "them to the data, retrain. Our expert can label any state, so this is cheap. Needs the time input, "
                  "or the labels contradict each other.", "A few rounds of rollouts plus training"],
                 ["Harder tests (P4.6 to P4.7)", "Held-out test split, unseen lighting, variants of client and arm.",
                  "Later"]], [W * 0.2, W * 0.55, W * 0.25]),
          Spacer(1, 8)]

story += [P("7. How you can help", "h2"),
          bullets(["<b>Run the Colab notebook</b> (<font face='Consolas'>colab_act_v4.ipynb</font>) when a new dataset "
                   "is uploaded, and paste the cell-7 summary table. Score the 40k checkpoint with cells 6 and 7.",
                   "<b>Decide the state design</b> for the retrain: add time-in-phase only, or also remove the gripper "
                   "pose? Read section 5 and Wen et al. 2020 first; it is a real design choice.",
                   "<b>Watch episodes.</b> Look at the per-episode traces in "
                   "<font face='Consolas'>p44_val_act_*.json</font> (field <font face='Consolas'>trace</font>: time, "
                   "phase, target error, target-to-gripper, expert-to-gripper in mm, close flag) and spot patterns, e.g. "
                   "do the pushing aborts happen at contact or earlier?",
                   "<b>Read the four core papers</b> below; each is short and maps straight onto something in this "
                   "project.",
                   "<b>Keep the rules</b>: one simulation run at a time, small runs (about 25 episodes), no long GPU "
                   "jobs on the laptop, no generated data in git."]),
          P("Commands you can run (WSL)", "h3"),
          Paragraph("bash projects/p4_vision/run_act_eval.sh &lt;pretrained_model&gt; &lt;tag&gt; 25 8<br/>"
                    "bash projects/p4_vision/run_act_eval.sh &lt;ckpt&gt; hyb 25 8 --expert-reach<br/>"
                    "bash projects/p4_vision/run_act_eval.sh &lt;ckpt&gt; shadow 2 2 --shadow --ens none", st["code"]),
          P("Reading list", "h3"),
          table([["Paper", "Why it matters here"],
                 ["Zhao et al. 2023, Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware (ACT)",
                  "The policy we use: chunking, temporal ensembling, CVAE."],
                 ["Ross, Gordon, Bagnell 2011, A Reduction of Imitation Learning to No-Regret Online Learning (DAgger)",
                  "Covariate shift and the standard fix, our P4.5."],
                 ["Laskey et al. 2017, DART: Noise Injection for Robust Imitation Learning",
                  "The noise we added to half the demos."],
                 ["Wen et al. 2020, Fighting Copycat Agents in Behavioral Cloning from Observation Histories",
                  "The copycat shortcut we hit (target = gripper + a little)."],
                 ["Codevilla et al. 2019, Exploring the Limitations of Behavior Cloning for Autonomous Driving",
                  "Why offline error and closed-loop success diverge."]], [W * 0.55, W * 0.45]),
          Spacer(1, 8)]

story += [P("8. Glossary", "h2"),
          table([["Term", "Meaning"],
                 ["Behaviour cloning", "Supervised learning of actions from expert demonstrations."],
                 ["Open loop / closed loop", "Evaluating on recorded expert states vs letting the policy act and see the "
                  "consequences."],
                 ["Covariate shift", "The policy's own errors lead it to states unlike the training data."],
                 ["Action chunk", "A short sequence of future actions predicted at once (here 20 frames = 2 s)."],
                 ["Temporal ensembling", "Averaging overlapping chunk predictions for the same frame."],
                 ["DART", "Training-data trick: perturb the executed action, keep the clean label."],
                 ["DAgger", "Iterative fix: roll out the policy, have the expert label the visited states, retrain."],
                 ["Oracle", "The classical controller with the true bar pose: the best case."],
                 ["E2 monitors", "Safety checks that abort an episode (back off, stow) on danger."],
                 ["track / clear_pred aborts", "E2 reasons: the bus left its station-keeping reference / a predicted "
                  "clearance violation."],
                 ["Bus frame", "Coordinates fixed to the servicer body; the policy's targets live there."]],
                [W * 0.3, W * 0.7])]


def on_page(c, doc):
    c.saveState()
    c.setFont("Arial", 7.5)
    c.setFillColor(MUTED)
    c.drawString(20 * mm, 10 * mm, "mjorbit P4.4 learner's guide")
    c.drawRightString(A4[0] - 20 * mm, 10 * mm, str(doc.page))
    c.restoreState()


doc = SimpleDocTemplate(str(OUT), pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=18 * mm,
                        bottomMargin=18 * mm, title="Learning to grasp from pixels: P4.4", author="mjorbit")
doc.build(story, onFirstPage=on_page, onLaterPages=on_page)
print(OUT)
