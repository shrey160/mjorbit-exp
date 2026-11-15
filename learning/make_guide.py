"""Build learning/mjorbit_guide.pdf: mission and system design guide (14 Nov 2026).

Replaces the P4.4 learner's guide (make_p44_guide.py), which presented ACT and a retrain as the
plan. Run with the Windows python that has reportlab:
    python learning/make_guide.py C:/Shrey_Projs/robotics/mjorbit
"""
import json
import sys
from pathlib import Path

from reportlab.graphics.shapes import Circle, Drawing, Line, PolyLine, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.fonts import addMapping
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (KeepTogether, ListFlowable, ListItem, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table,
                                TableStyle)

ROOT = Path(sys.argv[1])
OUT = ROOT / "learning" / "mjorbit_guide.pdf"
P5 = ROOT / "projects" / "p5_own_eyes"

F = "C:/Windows/Fonts/"
pdfmetrics.registerFont(TTFont("Arial", F + "arial.ttf"))
pdfmetrics.registerFont(TTFont("Arial-Bold", F + "arialbd.ttf"))
pdfmetrics.registerFont(TTFont("Arial-Italic", F + "ariali.ttf"))
pdfmetrics.registerFont(TTFont("Consolas", F + "consola.ttf"))
addMapping("Arial", 0, 0, "Arial")
addMapping("Arial", 1, 0, "Arial-Bold")
addMapping("Arial", 0, 1, "Arial-Italic")

INK, INK2, MUTED = colors.HexColor("#0b0b0b"), colors.HexColor("#52514e"), colors.HexColor("#898781")
GRID, AXIS, SURF = colors.HexColor("#e1e0d9"), colors.HexColor("#c3c2b7"), colors.HexColor("#fcfcfb")
S1, S2, S3 = colors.HexColor("#2a78d6"), colors.HexColor("#eb6834"), colors.HexColor("#1baf7a")
BOXBG, WARM, COOL = colors.HexColor("#f3f2ee"), colors.HexColor("#fdf0ea"), colors.HexColor("#eaf2fc")

st = dict(
    h1=ParagraphStyle("h1", fontName="Arial-Bold", fontSize=20, leading=24, textColor=INK, spaceAfter=6),
    sub=ParagraphStyle("sub", fontName="Arial", fontSize=11, leading=15, textColor=INK2, spaceAfter=12),
    h2=ParagraphStyle("h2", fontName="Arial-Bold", fontSize=14, leading=18, textColor=INK, spaceBefore=14, spaceAfter=6,
                      keepWithNext=1),
    h3=ParagraphStyle("h3", fontName="Arial-Bold", fontSize=11, leading=14, textColor=INK, spaceBefore=8, spaceAfter=3,
                      keepWithNext=1),
    p=ParagraphStyle("p", fontName="Arial", fontSize=10, leading=14.2, textColor=INK, spaceAfter=6, alignment=TA_LEFT),
    cap=ParagraphStyle("cap", fontName="Arial-Italic", fontSize=8.8, leading=12, textColor=INK2, spaceBefore=3, spaceAfter=10),
    box=ParagraphStyle("box", fontName="Arial", fontSize=9.6, leading=13.5, textColor=INK),
    cell=ParagraphStyle("cell", fontName="Arial", fontSize=8.6, leading=11.3, textColor=INK),
    cellb=ParagraphStyle("cellb", fontName="Arial-Bold", fontSize=8.6, leading=11.3, textColor=INK),
    code=ParagraphStyle("code", fontName="Consolas", fontSize=8.2, leading=11, textColor=INK, backColor=BOXBG,
                        borderPadding=5, spaceBefore=4, spaceAfter=10),
)
W = A4[0] - 40 * mm


def P(t, s="p"):
    return Paragraph(t, st[s])


def bullets(items):
    return ListFlowable([ListItem(P(i), leftIndent=10, value="•") for i in items], bulletType="bullet",
                        start="•", leftIndent=12, bulletFontName="Arial", bulletFontSize=9)


def callout(title, body, col=S1):
    t = Table([[P(f"<b>{title}</b><br/>{body}", "box")]], colWidths=[W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), BOXBG), ("LEFTPADDING", (0, 0), (-1, -1), 9),
                           ("RIGHTPADDING", (0, 0), (-1, -1), 9), ("TOPPADDING", (0, 0), (-1, -1), 7),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 7), ("LINEBEFORE", (0, 0), (0, -1), 2.5, col)]))
    return t


def table(rows, widths, head=True):
    data = [[P(c, "cellb" if (head and i == 0) else "cell") for c in r] for i, r in enumerate(rows)]
    t = Table(data, colWidths=[W * x for x in widths], repeatRows=1 if head else 0)
    t.setStyle(TableStyle([("LINEBELOW", (0, 0), (-1, 0), 0.8, AXIS), ("LINEBELOW", (0, 1), (-1, -1), 0.3, GRID),
                           ("VALIGN", (0, 0), (-1, -1), "TOP"), ("TOPPADDING", (0, 0), (-1, -1), 3),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]))
    return t


# ------------------------------------------------------------------ figures
def arrow(d, x0, y0, x1, y1):
    d.add(Line(x0, y0, x1, y1, strokeColor=INK2, strokeWidth=0.8))
    if abs(x1 - x0) >= abs(y1 - y0):
        s = 1 if x1 > x0 else -1
        d.add(Polygon([x1 + 2 * s, y1, x1 - 2 * s, y1 + 2.5, x1 - 2 * s, y1 - 2.5], fillColor=INK2, strokeColor=None))
    else:
        s = 1 if y1 > y0 else -1
        d.add(Polygon([x1, y1 + 2 * s, x1 - 2.5, y1 - 2 * s, x1 + 2.5, y1 - 2 * s], fillColor=INK2, strokeColor=None))


def conops(w=W, h=50 * mm):
    """Mission phases, left to right, with the range and the source of the client state."""
    d = Drawing(w, h)
    ph = [("1 Far approach", "30 m to 8 m", "filtered relative", "navigation", COOL),
          ("2 Hold", "8 m, on the", "provided state", "(Project 5)", WARM),
          ("3 Fly-in", "8 m to 1.25 m", "provided state", "(Project 5)", WARM),
          ("4 Reach", "arm to 12 cm", "camera network", "(coarse)", COOL),
          ("5 Approach, close", "last 12 cm", "palm camera", "network", COOL),
          ("6 Stabilise, move", "5 to 15 m", "own gyro and", "accelerometer", COOL),
          ("7 Release, retreat", "back to 8 m+", "simulator truth", "(demo only)", BOXBG)]
    ph[1] = ("2 Hold", "8 m, spin axis", "provided state", "(Project 5)", WARM)
    n, gx = len(ph), 7
    bw = (w - (n - 1) * gx) / n
    y, bh = h - 52, 46
    for i, (a, b, c, e, col) in enumerate(ph):
        x = i * (bw + gx)
        d.add(Rect(x, y, bw, bh, fillColor=col, strokeColor=AXIS, strokeWidth=0.6, rx=3, ry=3))
        d.add(String(x + bw / 2, y + bh - 12, a, fontName="Arial-Bold", fontSize=6.6, fillColor=INK, textAnchor="middle"))
        d.add(String(x + bw / 2, y + bh - 22, b, fontName="Arial", fontSize=6.6, fillColor=INK2, textAnchor="middle"))
        d.add(String(x + bw / 2, y + 13, c, fontName="Arial", fontSize=6.4, fillColor=INK, textAnchor="middle"))
        d.add(String(x + bw / 2, y + 5, e, fontName="Arial", fontSize=6.4, fillColor=INK, textAnchor="middle"))
        if i < n - 1:
            arrow(d, x + bw + 1, y + bh / 2, x + bw + gx - 2, y + bh / 2)
    # abort lane
    ya = y - 44
    d.add(Rect(bw + gx, ya, 4 * bw + 3 * gx, 26, fillColor=SURF, strokeColor=AXIS, strokeWidth=0.6, rx=3, ry=3,
               strokeDashArray=[3, 2]))
    xm = bw + gx + (4 * bw + 3 * gx) / 2
    d.add(String(xm, ya + 15, "Safety monitors run in phases 2 to 5.", fontName="Arial-Bold", fontSize=6.8,
                 fillColor=INK, textAnchor="middle"))
    d.add(String(xm, ya + 6, "Abort: open the jaws, pull back and stow the arm, back off towards 6 m.",
                 fontName="Arial", fontSize=6.8, fillColor=INK, textAnchor="middle"))
    for i in (1, 2, 3, 4):
        x = i * (bw + gx) + bw / 2
        arrow(d, x, y - 1, x, ya + 28)
    # legend
    for x, col, txt in ((0, COOL, "the servicer's own sensors"),
                        (w * 0.3, WARM, "state provided by the client (was simulator truth)"),
                        (w * 0.78, BOXBG, "still simulator truth")):
        d.add(Rect(x, 6, 9, 9, fillColor=col, strokeColor=AXIS, strokeWidth=0.5))
        d.add(String(x + 13, 8, txt, fontName="Arial", fontSize=6.8, fillColor=INK2))
    return d


def arch(w=W, h=58 * mm):
    """System architecture: sensors, estimation, guidance and control, actuators."""
    d = Drawing(w, h)
    cols = [("Sensors and links", ["Bus camera, 192 px", "Flash lidar, 128 px", "Wrist camera, 128 px",
                                   "Palm camera + light", "Joint encoders", "Gyro, accelerometer",
                                   "Star tracker", "Far-range relative nav", "Client data link"]),
            ("Estimation", ["Coarse bar-pose network", "(bus cam, lidar, wrist cam)", "Palm offset network",
                            "Agreement gate (4 cm)", "Far-approach filter", "(9 states, with bias)",
                            "Relative-motion filter", "(position, velocity,", "attitude, spin rate)",
                            "Stack identification", "(mass, inertia, centre)"]),
            ("Guidance and control", ["Far approach guidance", "Fly-in path planner", "(rotating client frame)",
                                      "Bus station keeping", "Arm: reach (joint path)", "Arm: approach (impedance)",
                                      "Jaw servo and brakes", "Stack detumble and move", "Safety monitors, abort",
                                      "Wheel unloading"]),
            ("Actuators", ["Thrusters", "0.75 N per axis", "Reaction wheels", "1 N m, 30 N m s", "Arm, 6 joints",
                           "60 to 12 N m peak", "Parallel-jaw gripper", "17.6 N clamp"])]
    gx = 16
    bw = (w - 3 * gx) / 4
    top, bot = h - 4, 4
    for i, (title, items) in enumerate(cols):
        x = i * (bw + gx)
        d.add(Rect(x, bot, bw, top - bot, fillColor=BOXBG, strokeColor=AXIS, strokeWidth=0.6, rx=3, ry=3))
        d.add(String(x + bw / 2, top - 13, title, fontName="Arial-Bold", fontSize=8.2, fillColor=INK, textAnchor="middle"))
        d.add(Line(x + 6, top - 18, x + bw - 6, top - 18, strokeColor=AXIS, strokeWidth=0.5))
        yy = top - 30
        for it in items:
            sub = it.startswith("(") or it[0].isdigit()
            d.add(String(x + (12 if sub else 7), yy, it, fontName="Arial", fontSize=6.9,
                         fillColor=INK2 if sub else INK))
            yy -= 9.6 if sub else 11.2
        if i < 3:
            arrow(d, x + bw + 2, (top + bot) / 2, x + bw + gx - 3, (top + bot) / 2)
    return d


def line_chart(series, xlab, ylab, xmin, xmax, ymax, yticks, xticks, w=W * 0.49, h=52 * mm):
    d = Drawing(w, h)
    L, B, R, T = 26, 26, 16, 36
    pw, ph = w - L - R, h - B - T
    X = lambda x: L + (x - xmin) / (xmax - xmin) * pw  # noqa: E731
    Y = lambda y: B + min(y, ymax) / ymax * ph  # noqa: E731
    for yt in yticks:
        d.add(Line(L, Y(yt), L + pw, Y(yt), strokeColor=GRID, strokeWidth=0.4))
        d.add(String(L - 4, Y(yt) - 3, f"{yt:g}", fontName="Arial", fontSize=7, fillColor=MUTED, textAnchor="end"))
    for xt in xticks:
        d.add(String(X(xt), B - 10, f"{xt:g}", fontName="Arial", fontSize=7, fillColor=MUTED, textAnchor="middle"))
    d.add(Line(L, B, L + pw, B, strokeColor=AXIS, strokeWidth=0.8))
    d.add(String(L + pw / 2, 3, xlab, fontName="Arial", fontSize=7.5, fillColor=INK2, textAnchor="middle"))
    d.add(String(2, h - 9, ylab, fontName="Arial", fontSize=7.5, fillColor=INK2))
    for k, (name, pts, col) in enumerate(series):  # legend above the plot
        xl = L + k * (pw / len(series))
        d.add(Line(xl, h - 20, xl + 12, h - 20, strokeColor=col, strokeWidth=1.8))
        d.add(String(xl + 15, h - 22.5, name, fontName="Arial", fontSize=7.3, fillColor=INK))
    for name, pts, col in series:
        d.add(PolyLine([v for x, y in pts for v in (X(x), Y(y))], strokeColor=col, strokeWidth=1.6, strokeLineJoin=1))
        for x, y in pts:
            d.add(Circle(X(x), Y(y), 2.3, fillColor=col, strokeColor=SURF, strokeWidth=0.8))
    return d


def bar_chart(rows, w=W, total=25):
    bh, gap = 11, 6
    h = len(rows) * (bh + gap) + 20
    d = Drawing(w, h)
    L, R = 190, 40
    top = h - 4
    pw = w - L - R
    for k in range(0, total + 1, 5):
        x = L + k / total * pw
        d.add(Line(x, 12, x, top, strokeColor=GRID, strokeWidth=0.4))
        d.add(String(x, 2, str(k), fontName="Arial", fontSize=7, fillColor=MUTED, textAnchor="middle"))
    for i, (name, v, col) in enumerate(rows):
        y = top - (i + 1) * (bh + gap) + gap
        d.add(String(L - 6, y + 3, name, fontName="Arial", fontSize=8, fillColor=INK, textAnchor="end"))
        d.add(Rect(L, y, max(v / total * pw, 1.5), bh, fillColor=col, strokeColor=None, rx=2, ry=2))
        d.add(String(L + v / total * pw + 4, y + 3, f"{v}/{total}", fontName="Arial", fontSize=8, fillColor=INK))
    return d


def side_by_side(a, b):
    t = Table([[a, b]], colWidths=[W * 0.5, W * 0.5])
    t.setStyle(TableStyle([("LEFTPADDING", (0, 0), (-1, -1), 0), ("RIGHTPADDING", (0, 0), (-1, -1), 0),
                           ("VALIGN", (0, 0), (-1, -1), "TOP")]))
    return t


fig_act = bar_chart([("Classical grasp, true bar pose", 25, S1), ("Classical grasp, camera pose estimate", 25, S1),
                     ("ACT, averaged predictions", 3, S2), ("ACT, 2 s chunks", 2, S2),
                     ("Classical reach, then ACT", 5, S2), ("ACT, newest prediction only", 0, S2)])
fig_spin = line_chart([("true pose", [(2, 25), (3, 19), (4, 11), (5, 11), (6, 11)], S1),
                       ("camera estimate", [(2, 25), (3, 13), (4, 11), (5, 11), (6, 11)], S2)],
                      "client spin rate (deg/s)", "grasps out of 25", 1.5, 6.5, 25, [0, 5, 10, 15, 20, 25], [2, 3, 4, 5, 6])
fig_mount = line_chart([("coarse network alone", [(1, 25), (3, 25), (5, 24), (10, 16)], S2),
                        ("with palm camera", [(1, 25), (5, 25), (10, 25)], S1)],
                       "camera mount error (mm per axis)", "grasps out of 25", 0, 11, 25, [0, 5, 10, 15, 20, 25],
                       [1, 3, 5, 10])

# ------------------------------------------------------------------ document
story = []
story += [P("mjorbit: mission and system design guide", "h1"),
          P("A servicer satellite that flies up to a client satellite, grasps its grapple bar with a robot arm, "
            "moves it and lets it go, all in simulation. This guide replaces the P4.4 guide of 13 Nov 2026. "
            "It describes the mission, the system, what has been measured, and what is still open. 15 Nov 2026.", "sub")]

story += [P("1. What changed since the last guide", "h2"),
          P("The last guide ended with a plan to retrain the ACT imitation policy. That plan was dropped. "
            "The table lists each change and the measurement behind it."),
          table([["Topic", "Last guide (13 Nov)", "Now (15 Nov)"],
                 ["Learned grasp policy (ACT)", "Fails in closed loop; retrain with a time input, then DAgger.",
                  "<b>Dropped.</b> Two more checks scored 0 of 25, one with a collision. The expert follows a clock, "
                  "so its labels never show how to correct an error."],
                 ["Where learning sits", "Whole behaviour, images to arm targets.",
                  "<b>Perception only.</b> Two networks estimate the bar pose; the classical controller does the motion."],
                 ["Last 12 cm", "Not treated separately.",
                  "<b>Palm camera</b> between the jaws, with a light, corrects the approach. 25 of 25 at ±10 mm "
                  "camera-mount error, against 16 of 25 without."],
                 ["Faster spin", "Assumed to be an arm problem.",
                  "<b>It is a thrust limit.</b> Above about 2.6°/s the servicer cannot hold station beside the bar."],
                 ["Whole mission", "Grasp and move only.",
                  "<b>Full demo</b> from 30 m: approach, camera-guided grasp, move, release, retreat."],
                 ["Client state for the servicer body", "Simulator truth, not discussed.",
                  "<b>Project 5.</b> The client reports its own state (cooperative client); a filter and re-tuned "
                  "monitors handle its errors, delay and gaps. <b>Working on 25 tune cases:</b> 25 of 25 grasps with "
                  "a realistic link, and all injected faults end safely (section 5)."]], [0.2, 0.33, 0.47]),
          Spacer(1, 4)]

story += [P("2. Mission design", "h2"),
          P("2.1 Goal and scope", "h3"),
          P("<b>Goal:</b> attach to a client satellite by its grapple bar, stop its rotation, move and turn it, then "
            "release it and withdraw, without any unintended contact."),
          bullets(["<b>Orbit:</b> circular, 400 km. Motion is described relative to the client in a frame that turns "
                   "with the orbit (0.065°/s).",
                   "<b>Client:</b> a box satellite, 1.5 × 1.0 × 1.0 m, 200 to 400 kg, with a grapple bar (30 cm long, "
                   "5 × 5 cm square section, 15 cm off one face). It may spin at up to 2°/s about any axis.",
                   "<b>Servicer:</b> about 150 kg, sized to launch as a secondary payload (61 × 71 × 97 cm envelope), "
                   "with one 6-joint arm and a two-jaw gripper.",
                   "<b>Claim:</b> simulation only. No real images, no hardware. Results are on held-out random seeds."]),
          KeepTogether([P("2.2 Which kind of client", "h3"),
          P("Two questions describe a client: does it report its state and hold still (cooperative), and was it built "
            "to be serviced (prepared)? The simple case is chosen first; the others are extensions."),
          table([["Type", "Reports its state", "Grapple feature", "Real examples", "In mjorbit"],
                 ["A. Cooperative, prepared", "yes", "yes", "Orbital Express; new satellites with docking plates",
                  "<b>Chosen now</b> (Project 5)"],
                 ["B. Cooperative, unprepared", "yes", "no", "MEV-1 docking to Intelsat 901", "Not planned"],
                 ["C. Silent, prepared", "no", "yes", "A dead satellite that carries a docking plate",
                  "First extension: the servicer's own cameras see the whole client"],
                 ["D. Silent, unprepared", "no", "no", "Dead legacy satellites, rocket bodies",
                  "Last extension: needs a gripper for the launch adapter ring"]], [0.2, 0.12, 0.12, 0.28, 0.28])]),
          Spacer(1, 4),
          P("2.3 Mission phases", "h3"), conops(),
          P("Figure 1. The seven phases. The colour shows where the client's state comes from in each phase.", "cap"),
          table([["Phase", "What happens", "Ends when"],
                 ["1 Far approach", "Impulsive guidance from 20 to 50 m to a hold point 8 m out on the client's spin "
                  "axis, around a keep-out zone. Navigation: position and velocity with 5 cm and 1 mm/s noise, through a "
                  "9-state filter that also estimates a constant disturbance.",
                  "Settled at the hold point."],
                 ["2 Hold", "Waits on the spin axis, where the client's rotation does not move the servicer. In Project 5 "
                  "the filter settles here for 30 s and the fly-in is planned from the estimated spin axis.",
                  "Filter settled."],
                 ["3 Fly-in", "A path fixed in the client's rotating frame: along the axis, out to the bar's radius, "
                  "then in to the pre-grasp point 1.25 m from the bar. Peak 5 cm/s. The path slows itself when the "
                  "thrust needed would pass 0.65 N.", "At the pre-grasp point, turning with the client."],
                 ["4 Reach", "The arm unfolds over 20 s to 12 cm in front of the bar. Target from the coarse camera "
                  "network.", "20 s elapsed."],
                 ["5 Approach and close", "The gripper moves in at 1 cm/s under impedance control (it gives way on "
                  "contact). Target from the palm camera. Jaws close when the bar is within 1 cm.",
                  "Both pads on the bar."],
                 ["6 Stabilise and move", "Arm and jaws are braked. Wheels stop the rotation; thrust and wheel pulses "
                  "identify the mass, inertia and centre of mass of the joined pair; then it moves 5 to 15 m and turns "
                  "30 to 120°.", "Within 0.2 m, 2 mm/s, 0.5°, 0.01°/s."],
                 ["7 Release and retreat", "Jaws open, the gripper pulls back 20 cm, the bus backs away, the arm stows.",
                  "Servicer clear and stowed."]], [0.17, 0.6, 0.23]),
          Spacer(1, 4),
          P("2.4 Requirements", "h3", ),
          table([["Requirement", "Value", "Fixed"],
                 ["Capture", "Both pads on the bar after closing and after the hold; grip point within 2 cm of the bar "
                  "axis; bar slip below 2 mm/s.", "Project 3"],
                 ["Client push at capture", "At most 2 mm/s.", "12 Nov, before any estimator result"],
                 ["Unintended contact", "None. Only gripper pads on the bar are allowed.", "Project 3"],
                 ["Safe outcome", "Success, or an abort that reaches the safe state (backed off beyond 3.2 m on the way to 6 m, arm "
                  "stowed, jaws open, not closing).", "Project 3, step E2"],
                 ["Move accuracy", "0.2 m, 2 mm/s, 0.5°, 0.01°/s of the command.", "Project 3"],
                 ["Client spin", "0 to 2°/s, any axis.", "Thrust limit near 2.6°/s"],
                 ["Provided client state (Project 5)", "Position within 60 mm per axis, attitude within 0.05°, "
                  "one message per second, up to 0.3 s late. The arm cameras take over the position near the bar.",
                  "Measured 14 to 15 Nov (section 5)"]], [0.25, 0.53, 0.22]),
          Spacer(1, 4),
          P("2.5 Safety design", "h3"),
          P("Monitors run every control step (100 Hz) from the hold to the capture. Any trip starts the same abort, "
            "chosen so it does not depend on the client moving as expected."),
          table([["Monitor", "Trips when"],
                 ["Predicted clearance", "The bus, coasting from now for 30 s, would come within 0.3 m of the turning client. "
                  "In Project 5 an allowance for the estimate's uncertainty is taken off first."],
                 ["Tracking", "The bus is more than 0.25 m or 8° from its reference near the client."],
                 ["Rate anomaly", "The client's rotation rate jumps by more than 0.1°/s within 2 s (it was kicked)."],
                 ["Thruster check", "Measured acceleration disagrees with commanded thrust by more than 0.04 N."],
                 ["Stale link (Project 5)", "The newest client message is more than 3 s old."],
                 ["Navigation anomaly (Project 5)", "A client attitude message disagrees with the filter's prediction "
                  "far beyond its noise. This catches a kicked client within two messages."],
                 ["Contact", "Anything other than a pad touches the client."],
                 ["Grip slip", "After capture: the bar shifts 6 mm, slips at 3 mm/s, or a pad lifts for 0.2 s. "
                  "Response: re-clamp."]], [0.22, 0.78]),
          Spacer(1, 4)]

story += [P("3. System design", "h2"),
          P("3.1 Spacecraft", "h3"),
          table([["Item", "Servicer", "Client"],
                 ["Mass", "150 kg (bus 133.6 kg, arm and gripper 16.6 kg)", "200 to 400 kg, randomised and unknown"],
                 ["Size", "Bus 0.5 × 0.6 × 0.9 m; arm folds flat on one face", "1.5 × 1.0 × 1.0 m"],
                 ["Translation", "Thrusters, 0.75 N per axis (5 mm/s² on the servicer alone)", "None (passive)"],
                 ["Rotation", "Reaction wheels, 1 N m and 30 N m s per axis; thruster couples of 0.5 N m unload them",
                  "Free spin up to 2°/s, slowly wobbling"],
                 ["Arm", "6 joints: shoulder yaw and pitch, elbow, three wrist joints. Links 0.55 and 0.45 m. Geared "
                  "motors, 60 / 60 / 40 / 12 / 12 / 12 N m peak, with friction and rotor inertia modelled", "—"],
                 ["Gripper", "Two flat pads on opposing slides, 17.6 N clamp, locked after capture",
                  "Grapple bar, 30 cm, square section"]], [0.14, 0.5, 0.36]),
          Spacer(1, 4),
          KeepTogether([P("3.2 Architecture", "h3"), arch()]),
          P("Figure 2. System blocks. Data flows left to right. The controller runs at 100 Hz, cameras and networks at "
            "10 Hz, physics at 500 Hz.", "cap"),
          P("3.3 Who senses what", "h3"),
          P("This is the honest list of what each function reads. Removing the last \"simulator truth\" rows is the "
            "purpose of Project 5 and the items after it."),
          table([["Function", "Reads today", "Planned"],
                 ["Far approach", "Simulated relative navigation with noise, through a filter", "No change"],
                 ["Choice of hold point (spin axis)", "Simulator truth", "Client's reported spin"],
                 ["Fly-in and station keeping", "Client's link through the filter; arm cameras for position near "
                  "the bar (Project 5)", "Silent client: own camera and lidar"],
                 ["Clearance, tracking and rate monitors", "Filter output with an allowance; simulator truth only "
                  "during the 30 s wait at the hold point", "Filter output throughout"],
                 ["Abort back-off", "Filter output (Project 5)", "No change"],
                 ["Arm reach", "Coarse network on bus camera, lidar and wrist camera", "No change"],
                 ["Arm approach and close", "Palm camera network, checked against the coarse network", "No change"],
                 ["After capture", "Own gyro and accelerometer with noise; identified mass properties", "No change"],
                 ["Release and retreat", "Simulator truth (written for the demo)", "Later"]], [0.27, 0.36, 0.37]),
          Spacer(1, 4),
          P("3.4 Software map", "h3"),
          table([["Folder", "Holds"],
                 ["simulator/", "mjorbit: MuJoCo with orbital dynamics. Do not edit."],
                 ["projects/p2_approach/", "Far approach: environment, guidance, navigation filter."],
                 ["projects/p3_capture/", "Fly-in, grasp, monitors, stack control, mission runs, replay. Frozen: new "
                  "work hooks in from outside."],
                 ["projects/p4_vision/", "Cameras and scene, pose networks, palm camera, evaluation runs, the dropped "
                  "ACT pipeline."],
                 ["projects/p5_own_eyes/", "Tolerance sweeps, relative-motion filter, link model, fly-in speed limit, "
                  "fault runs, the Project 5 spec."],
                 ["learning/", "This guide, research notes with sources, lessons learned."],
                 ["CONTEXT.md", "The running log. Every session starts by reading its first section."]], [0.27, 0.73]),
          Spacer(1, 4)]

story += [P("4. Where learning helped, and where it did not", "h2"),
          P("4.1 The end-to-end policy failed", "h3"),
          P("ACT (a transformer that predicts 2 s of arm targets from images) reached 5 to 9 mm error when scored "
            "against recorded demonstrations. Driving the arm itself, it failed."),
          fig_act,
          P("Figure 3. Successful grasps out of 25 validation cases. Blue: classical controller. Orange: ACT variants.", "cap"),
          bullets(["<b>Cause:</b> the classical expert moves on a schedule (a 20 s reach, then 1 cm/s). Its target "
                   "does not depend on where the arm actually is. A policy that copies it learns \"target = gripper "
                   "plus a little\" and never learns to recover once it lags.",
                   "<b>Averaging hid the problem:</b> the raw predictions jump by tens of millimetres from frame to "
                   "frame. Without averaging the arm either stays stowed or jerks into an abort.",
                   "<b>More training would not fix it:</b> the approach error stayed at 5 to 7 mm from 16,000 to 40,000 "
                   "steps.",
                   "<b>Lesson:</b> a low error on recorded data says little about behaviour when the policy drives. "
                   "Always run it closed loop."]),
          P("4.2 Measure where the classical system breaks before choosing a task", "h3"),
          side_by_side(fig_spin, fig_mount),
          P("Figure 4. Left: grasps against client spin rate. Right: grasps against camera-mount error, without and "
            "with the palm camera. 25 validation cases per point.", "cap"),
          bullets(["<b>Spin:</b> the grasp holds to 2°/s and falls off above. Tracing single cases showed the thrust "
                   "needed to circle with the bar is 0.95 to 1.1 N at 3°/s, against 0.75 N available. No arm policy "
                   "can supply thrust, so this task was withdrawn before any training.",
                   "<b>Calibration:</b> the coarse network was trained with cameras mounted within 1 mm. At ±10 mm and "
                   "±1° it grasps 16 of 25. This was the real gap."]),
          P("4.3 The palm camera correction", "h3"),
          bullets(["<b>Why a new camera:</b> the wrist camera sits above the jaws and never sees the pads. A camera "
                   "between the jaws sees the bar and both pads through the whole approach.",
                   "<b>What is learned:</b> a small network maps the palm image to the bar's offset from the grip point. "
                   "The arm's own joint angles turn that into a bar position, so camera-mount error on the bus no "
                   "longer matters.",
                   "<b>Gate:</b> the palm reading is used only when the coarse network puts the bar in the palm view "
                   "and the two agree within 4 cm. Without the gate the network answered \"centred\" with the bar out "
                   "of view, and 7 to 9 of 25 runs collided.",
                   "<b>Light:</b> the gripper shadows the bar in the last centimetres. A small light beside the camera "
                   "and twice the data took the result from 22 to 25 of 25.",
                   "<b>Result:</b> 25 of 25 at ±1, ±5 and ±10 mm; on 25 fresh cases at ±10 mm, 25 of 25 against "
                   "16 of 25. No collisions, no aborts.",
                   "<b>Test cases (run once, 15 Nov):</b> 25 of 25 with the palm camera against 10 of 25 without. "
                   "With sun directions never seen in training: 25 of 25 against 17 of 25. No trend with spin "
                   "rate from 0 to 2°/s.",
                   "<b>Not learned:</b> the bar's tilt and its position along its own length still come from the "
                   "coarse network."]),
          Spacer(1, 2)]

filt_rows = None
fj = P5 / "p5_filt_closed.json"
if fj.exists():
    filt_rows = [["Measurement error fed to the filter", "Grasps", "Safe aborts", "Filter error before contact "
                  "(position, velocity, attitude, rate; rms)", "Spin axis after 30 s"]]
    fd = json.loads(fj.read_text())
    for s in fd["summary"]:
        n_ab = sum(s["aborts"].values())
        fe = " / ".join(f"{s['f_prox_' + k]['rms']:.{dg}f} {u}" for k, dg, u in
                        (("p", 1, "mm"), ("v", 2, "mm/s"), ("a", 2, "°"), ("w", 3, "°/s")) if "f_prox_" + k in s)
        ae = sorted(e["pre_axis_err_deg"] for e in fd["episodes"] if e["cond"] == s["cond"] and e["spin"] >= 0.1)
        ax = f"{ae[len(ae) // 2]:.1f}° median, {ae[-1]:.1f}° worst"
        filt_rows.append([f"{s['meas_mm']:g} mm, {s['meas_deg']:g}°", f"{s['success']}/{s['n']}", str(n_ab), fe, ax])

story += [P("5. Project 5: the servicer body's knowledge of the client", "h2"),
          P("Until now the bus read the client's exact state from the simulator. Project 5 replaces that. The first "
            "step measured how much error the bus can stand, by corrupting the state it reads (25 cases per row; the "
            "arm kept the true pose so only the bus effect shows)."),
          table([["Error fed to the bus (1σ)", "Grasps", "Safe aborts", "Collisions", "Fly-in propellant (mm/s of Δv)"],
                 ["none", "25/25", "0", "0", "419"],
                 ["position 10 / 30 / 100 mm", "25 / 23 / 0", "0 / 2 / 25", "0", "423 / 452 / —"],
                 ["attitude 0.5 / 1 / 3°", "25 / 25 / 0", "0 / 0 / 25", "0", "454 / 508 / —"],
                 ["velocity 1 / 3 / 10 mm/s", "25 / 19 / 4", "0 / 6 / 19", "0 / 0 / 1", "450 / 583 / 653"],
                 ["spin rate 0.01 / 0.03 / 0.1°/s", "25 / 25 / 25", "0", "0", "551 / 891 / 2651"],
                 ["delay 0.1 / 0.3 / 1.0 s", "25 / 25 / 25", "0", "0", "419 / 419 / 419"],
                 ["all at once: 10 mm, 0.5°, 1 mm/s, 0.01°/s, 0.2 s", "24", "1", "0", "576"],
                 ["the same, monitors also on the corrupted state", "16", "9", "0", "576"]],
                [0.36, 0.14, 0.14, 0.12, 0.24]),
          Spacer(1, 6),
          bullets(["<b>The control loops tolerate about 10 mm, 1° and 1 mm/s.</b> Velocity error is the tightest and "
                   "the only one that caused a collision.",
                   "<b>The monitors are stricter than the controller.</b> On an estimate the controller handles, the "
                   "predicted-clearance monitor aborts 9 of 25 fly-ins. It needs an allowance for estimate error.",
                   "<b>Spin-rate noise costs propellant, not grasps,</b> because the fly-in differentiates the rate.",
                   "<b>Delay up to 0.3 s does not matter.</b>"]),
          P("5.1 The relative-motion filter", "h3"),
          P("The filter takes pose data at 10 Hz, 0.15 s late, and outputs position, velocity, attitude and spin rate "
            "at 100 Hz, carried forward to the present. Two design points came from testing:"),
          bullets(["<b>It tracks the servicer's centre of mass, not its body.</b> The centre of mass accelerates by "
                   "exactly thrust divided by mass whatever the arm does, so the filter can smooth hard. Velocity "
                   "error in the fly-in fell from 2.1 to 0.7 mm/s.",
                   "<b>Slow drift in the input matters more than noise.</b> If half of the input error wanders over "
                   "about 5 s, velocity and rate errors are three to five times larger than with pure noise."])]
if filt_rows:
    story += [table(filt_rows, [0.2, 0.1, 0.1, 0.36, 0.24]),
              P("Table: bus flying on the filter, fly-in planned from the estimated spin axis, 25 tune cases per row. "
                "Monitors and arm on the true state. All aborts were the predicted-clearance monitor during the fly-in. "
                "Spin-axis error leaves out one case that spins at 0.03°/s, where the axis has no meaning.", "cap")]
else:
    story += [callout("Status", "A 4-case check with 10 mm and 0.5° input error gave 4 of 4 grasps with the bus flying "
                      "on the filter and the fly-in planned from the estimated spin axis (within 2.9° median, 7.5° "
                      "worst). Filter error before contact: 8.6 mm, 0.36 mm/s, 0.31°, 0.008°/s. The 25-case run was "
                      "still in progress when this guide was built.", S2)]
story += [P("5.2 Monitors on the estimate", "h3"),
          bullets(["<b>Why fly-ins were aborting:</b> the planned path left the clearance monitor only 3 cm of margin, "
                   "so 3 cm of tracking error tripped it. None of the traced aborts was a real small clearance.",
                   "<b>Change:</b> the monitors read the filter. The predicted clearance loses an allowance of two "
                   "standard deviations of the estimate, and the path is planned to keep that allowance plus 8 cm.",
                   "<b>New monitor:</b> abort when the newest client message is older than 3 s. A 1 s gap is "
                   "ridden through; a 6 s gap gives a safe abort."]),
          P("5.3 What a cooperative client can report", "h3"),
          P("From flown missions: relative satellite navigation gives about 10 cm in three dimensions on board, which "
            "is 60 mm per axis and drifts slowly; 20 mm is a good case. A star tracker gives 0.05°. Messages arrive "
            "at 1 to 5 per second. With the arm on the true bar pose, a 20 mm link gave 25 of 25 and a 60 mm link 23 "
            "of 25: position is the limit, not attitude, message rate or delay."),
          P("5.4 The whole chain on the servicer's own information", "h3"),
          P("Bus and monitors on the link filter, arm on the two camera networks with ±10 mm mount error. "
            "25 tune cases per row."),
          table([["Link", "Design", "Grasps", "Largest push on client", "Fly-in time, one leg / three legs"],
                 ["60 mm, 1 Hz", "link only", "20/25", "7.70 mm/s", "408 / 412 s"],
                 ["60 mm, 1 Hz", "+ hand-over to the arm cameras", "23/25", "0.84 mm/s", "408 / 412 s"],
                 ["60 mm, 1 Hz", "+ drift filter and speed-limit fly-in", "25/25", "1.04 mm/s", "242 / 348 s"]],
                [0.14, 0.34, 0.12, 0.2, 0.2]),
          Spacer(1, 6),
          bullets(["<b>Hand-over:</b> two seconds after the arm cameras see the bar, they replace the link as the "
                   "filter's position source. The bus moves over slowly and waits (74 s median) before the arm "
                   "reaches. This removed every failure at the grasp.",
                   "<b>Drift filter:</b> the link error wanders over about a minute, and the first filter read that "
                   "as velocity (2 to 4 mm/s). Carrying the drift as extra states cut the fly-in velocity error to "
                   "1 mm/s.",
                   "<b>Speed limit:</b> at each point of the fly-in the speed is the largest one for which a 30 s "
                   "coast keeps the clearance margin. It replaced stretching the whole path and made the fly-in "
                   "shorter."]),
          P("5.5 Faults", "h3"),
          P("One fault per case at a random time in the fly-in, reach or approach. 25 tune cases per row, 60 mm link, "
            "aborts flown on the estimate."),
          table([["Fault", "Aborted", "Captured", "Collisions", "Detection delay (median / max)"],
                 ["Thruster stuck open (0.1 to 0.4 N)", "25", "0", "0", "0.16 / 0.28 s"],
                 ["Thruster weak (0 to 30% left)", "13", "12", "0", "12.8 / 91.7 s"],
                 ["Spin axis wrong by 5 to 15°", "0", "25", "0", "not needed"],
                 ["Client kicked 0.5 to 2°/s, first monitors", "18", "6", "1", "9.6 / 50.8 s"],
                 ["Client kicked, with the navigation-anomaly monitor", "25", "0", "0", "1.2 / 1.7 s"]],
                [0.42, 0.12, 0.12, 0.12, 0.22]),
          Spacer(1, 6),
          bullets(["<b>The kick was a real gap.</b> The rate monitor reads the filter, and the filter follows a kick "
                   "slowly. Kicks during the approach gave one collision and pushes up to 15.7 mm/s with no abort.",
                   "<b>Fix:</b> abort when an attitude message disagrees with the filter's prediction by more than "
                   "60 times its expected square. Fault-free runs reach 17 at most; kicks give 98 to 1431.",
                   "<b>All 63 aborts reached the safe state</b> within 46 s, steering on the estimate, which was up "
                   "to 178 mm off when the abort began."]),
          P("5.6 Still open in Project 5", "h3"),
          bullets(["All of this is on tune cases. The untouched test cases have not been run.",
                   "Kicks weaker than 0.5°/s were not tried.",
                   "The monitors still read the true state during the 30 s wait at the hold point.",
                   "The link delay (0.3 s) is an assumption, not a sourced figure."])]

story += [P("6. Evidence table", "h2"),
          P("What has been run, on how many cases, and what it does not show."),
          table([["Claim", "Evidence", "Not shown"],
                 ["Whole mission works with faults injected", "500 validation and 300 test missions (first compact "
                  "servicer arm): no fault 395/399 and 242/244; all outcomes safe 99.8% and 100%.",
                  "Realistic arm: 25 missions only (24 success, 1 safe abort)."],
                 ["Grasp from cameras equals grasp from true pose", "25/25 both, realistic arm, validation cases.",
                  "Test cases not run for the realistic arm."],
                 ["Palm camera removes the calibration sensitivity", "±10 mm mounts: 25/25 on 25 test cases (10/25 "
                  "without), 25/25 with unseen sun directions (17/25 without); 75 distinct cases in all, no trend "
                  "with spin.", "Bar tilt from the palm camera; arm calibration error."],
                 ["Full mission from 30 m with release", "One mission, shown in the replay page. Release pushed the "
                  "client 0.013 mm/s.", "A batch; a wheel model during release."],
                 ["Fly-in and grasp with no simulator truth in the bus or arm", "25 tune cases, 60 mm link, camera "
                  "hand-over: 25/25, no aborts, no collisions.", "Test cases; the 30 s wait at the hold point."],
                 ["Faults end safely on the estimate", "100 faulted tune cases: 0 collisions, 63 aborts, all reaching "
                  "the safe state.", "Test cases; kicks below 0.5°/s."]], [0.27, 0.42, 0.31]),
          Spacer(1, 4)]

story += [P("7. Limits and roadmap", "h2"),
          P("Known limits", "h3"),
          bullets(["Spin above 2°/s: needs a new bus mode that follows the bar only partly, or more thrust.",
                   "The palm network does not estimate the bar's tilt.",
                   "Simulation only: rendered images, no real camera data.",
                   "The client is one shape with one bar. Other fixtures need other grippers."]),
          P("Order of work", "h3"),
          table([["Step", "What", "Status"],
                 ["1", "Project 5 for a cooperative client: filter, monitors, link model, closed loop, faults, test",
                  "Done on tune cases; test cases open"],
                 ["2", "Make the palm result solid: test cases once, unseen sun, by spin rate, bar tilt",
                  "Done except bar tilt"],
                 ["3", "Release on a batch of missions with a wheel model", "Open"],
                 ["4", "Silent client: the servicer's own cameras see the whole client", "Planned (old Project 5 plan)"],
                 ["5", "Faster spin; force and contact sensing; learning for gentler contact", "Later"]],
                [0.08, 0.64, 0.28]),
          Spacer(1, 4)]

story += [P("8. How you can help", "h2"),
          bullets(["<b>Watch the replay page</b> and note anything that looks wrong; the paused-orbit bug was found "
                   "that way.",
                   "<b>Check the link delay.</b> The accuracy figures now have sources (RESEARCH_SERVICING.md, "
                   "section Provided state); the 0.3 s delay is still an assumption.",
                   "<b>Read SPEC_5</b> and question the requirements table: is 2 mm/s push the right limit for a 300 kg "
                   "client?",
                   "<b>Keep the rules:</b> one simulation run at a time, about 25 cases per run, no long GPU jobs on "
                   "the laptop, no generated data in git, update CONTEXT.md before a run."]),
          P("Commands (PowerShell; each call runs inside WSL)", "h3"),
          Paragraph("wsl -d Ubuntu -- bash /mnt/c/Shrey_Projs/robotics/mjorbit/projects/p5_own_eyes/run_p5.sh "
                    "noise_tol.py --conds base:0,0,0,0,0 --n 25 --workers 8 --out &lt;file.json&gt;<br/>"
                    "wsl -d Ubuntu -- bash .../p5_own_eyes/run_p5cam.sh cam_eval.py --conds nom:60,0.05,1,0.3 --cam-fix "
                    "--n 25 --workers 8 --out &lt;absolute path&gt;.json<br/>"
                    "(add --faults client_kick thruster_stuck for fault cases)<br/>"
                    "wsl -d Ubuntu -- bash .../p4_vision/run_palm_build.sh eval 10 25 --palm-no-rot<br/>"
                    "python learning/make_guide.py C:/Shrey_Projs/robotics/mjorbit", st["code"])]

story += [P("9. Glossary", "h2"),
          table([["Term", "Meaning"],
                 ["Servicer / client", "The spacecraft with the arm / the satellite being serviced."],
                 ["Grapple bar", "A handle on the client made for a gripper."],
                 ["Cooperative / prepared", "The client reports its state and holds still / it carries a grapple feature."],
                 ["Hold point", "A waiting position 8 m from the client on its spin axis."],
                 ["Fly-in", "The path from the hold point to the pre-grasp point, drawn in the client's turning frame."],
                 ["Station keeping", "Holding a fixed position relative to the (turning) client."],
                 ["Impedance control", "Arm control that behaves like a spring and damper, so it gives way on contact."],
                 ["Stack", "Servicer and client joined after capture."],
                 ["Closed loop / open loop", "The policy or estimate drives the system / it is only scored on recordings."],
                 ["ACT", "Action Chunking with Transformers, the imitation policy that was tried and dropped."],
                 ["Coarse network / palm network", "Bar pose from bus camera, lidar and wrist camera / bar offset from "
                  "the camera between the jaws."],
                 ["Gate", "A check that two estimates agree before the finer one is used."],
                 ["Filter", "An algorithm that blends late, noisy measurements with a motion model into a smooth estimate."],
                 ["Allowance", "Margin taken off a monitor's value to cover the uncertainty of the estimate it reads."],
                 ["Hand-over", "Switching the filter's position source from the client's link to the arm cameras."],
                 ["Innovation", "The difference between a new measurement and what the filter predicted."],
                 ["Monitor / abort", "A safety check that runs every step / the fixed safe response when one trips."],
                 ["Tune, validation, test cases", "Random seeds for adjusting settings / for decisions / used once at the end."],
                 ["Δv", "Total velocity change from thrust; a measure of propellant used."],
                 ["1σ", "One standard deviation of an error."]], [0.27, 0.73]),
          P("Sources for the research statements are listed in learning/RESEARCH_SERVICING.md.", "cap")]


def on_page(c, doc):
    c.saveState()
    c.setFont("Arial", 7.5)
    c.setFillColor(MUTED)
    c.drawString(20 * mm, 10 * mm, "mjorbit mission and system design guide, 15 Nov 2026")
    c.drawRightString(A4[0] - 20 * mm, 10 * mm, str(doc.page))
    c.restoreState()


doc = SimpleDocTemplate(str(OUT), pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=18 * mm,
                        bottomMargin=18 * mm, title="mjorbit: mission and system design guide", author="mjorbit")
doc.build(story, onFirstPage=on_page, onLaterPages=on_page)
print(OUT)
