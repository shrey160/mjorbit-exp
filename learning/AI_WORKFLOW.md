# Learn the concepts while using AI coding tools

The goal is to become able to specify, predict, inspect, and explain a spacecraft-control experiment. You can let AI write much of the implementation while still owning the engineering decisions.

## The working loop

1. Describe the physical question in ordinary language.
2. Predict the outcome before asking for implementation.
3. Write the smallest task contract: inputs with frames/units, bounded outputs, reset conditions, success, and failure.
4. Ask AI for one small implementation or diagnostic. Request the assumptions and a simple verification experiment.
5. Inspect the result, plot physical quantities, and compare with your prediction.
6. Explain the discrepancy in your own words, then ask AI to challenge that explanation.
7. Change one important factor and repeat. Keep the experiment record.

## Useful requests to give an AI assistant

### To learn a concept

"Explain angular velocity versus orientation using my spacecraft project. Ask me to predict a torque-pulse experiment before showing the answer. Give me one simple numerical example, and identify each frame and unit."

### To specify before implementing

"Help me define the detumbling task. List observations, actions, reset distribution, actuator bounds, success, failures, and metrics. Identify hidden assumptions. Do not implement until the physical contract is explicit."

### To build a small piece

"Implement only the agreed one-axis torque-pulse experiment using the project's pinned versions. Explain the physical role of each component and show a plot that can reveal a sign or unit error. Keep unrelated changes out of this experiment."

### To debug learning

"Success is flat although return is rising. Here are reward components, angular rates, actions, resets, and the success definition. Propose three plausible causes and the smallest experiment that distinguishes them. Avoid changing several hyperparameters at once."

### To check understanding

"Quiz me about this run without giving the answers immediately. Ask why the spacecraft overshoots, why its base moves when the arm moves, and what evidence would disprove my explanation."

## What you should be able to review

You do not need to memorize a simulator API. You do need to recognize a wrong frame, wrong units, impossible command, incomplete observation, unrealistic reset, and success metric that rewards the wrong behaviour.

Before trusting generated training code, ask the assistant to demonstrate:

- The reset really changes only the intended initial conditions and clears relevant actuator/history state.
- A positive command produces the expected signed motion on the expected axis.
- Action bounds correspond to actual physical limits.
- A known successful and a known failed trajectory produce the correct success/failure classification.
- Reward components do not reward contact without retention or proximity without braking.
- GPU data is current, and a host-side read is not accidentally inspecting an old mirror.
- Episode endings and training bootstrapping follow the chosen library's conventions.

These are targeted behavioural checks. Do not build a large testing framework before the first understandable experiment.

## Protect the learning objective

Ask for explanations and evidence before large rewrites. Save your prediction before AI sees the result. Keep a short glossary in your own words. If you cannot explain why an observation or reward exists, revisit it before adding more complexity.

You can deliberately request a hint, a diagram, a worked example, then an implementation in that order. For maths, understand the meaning of a formula and verify one numerical case before pursuing a derivation.

## Weekly check-in

Answer these without looking at code:

1. What did I expect physically, and why?
2. What did the experiment show?
3. What observation rules out my easiest alternative explanation?
4. Which assumption limits the result?
5. What is the smallest next experiment?

If the answers are clear, substantial AI coding assistance is compatible with the learning goal.
