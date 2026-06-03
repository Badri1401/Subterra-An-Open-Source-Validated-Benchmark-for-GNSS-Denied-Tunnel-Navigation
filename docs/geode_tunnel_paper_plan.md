# GEODE Underground Tunnel — RSS 2026 Workshop Paper Plan
### Workshop 31: Planning and Control with Imperfect Sensors and Perception
### RSS 2026 | July 17, 2026 | Submission Deadline: June 5, 2026

---

## Table of Contents
1. [What is GEODE?](#1-what-is-geode)
2. [Project Overview](#2-project-overview)
3. [Workshop Selection & Justification](#3-workshop-selection--justification)
4. [Core Paper Argument](#4-core-paper-argument)
5. [The Three Contributions](#5-the-three-contributions)
6. [Validation Strategy — All Three Methods](#6-validation-strategy--all-three-methods)
7. [Navigation Study — The Scientific Experiment](#7-navigation-study--the-scientific-experiment)
8. [Full Paper Structure (2-Page Abstract)](#8-full-paper-structure-2-page-abstract)
9. [Task Breakdown & Timeline](#9-task-breakdown--timeline)
10. [Key Terminology to Use in the Paper](#10-key-terminology-to-use-in-the-paper)
11. [Arguments Against Common Review Criticisms](#11-arguments-against-common-review-criticisms)

---

## 1. What is GEODE?

### Definition
GEODE (**GEometrically DEgenerate**) is a real-world, publicly available LiDAR dataset created specifically to benchmark SLAM (Simultaneous Localization and Mapping) algorithms in environments where standard localization breaks down.

### Key Facts About GEODE
- **64 trajectories** spanning over **64 km** of real-world data
- Collected across **7 different environment types**:
  - Subway tunnels
  - Urban tunnels
  - Flat surfaces
  - Stairways
  - Rough off-road terrain
  - Inland waterways
  - Bridges
- **Sensor suite per collection run**:
  - Multiple LiDAR types (rotating and non-repetitive)
  - Stereo cameras
  - IMU (Inertial Measurement Unit)
- **Four collection platforms**: handheld unit, UGV (Unmanned Ground Vehicle), sailboat, traditional vehicle
- **Ground truth trajectories** provided for every sequence
- Publicly available at: `https://github.com/PengYu-Team/GEODE_dataset`

### What is "Geometric Degeneracy"?
This is the most important concept in your paper. Geometric degeneracy means:
> The environment looks the same everywhere — featureless walls, uniform cross-sections, no distinct visual or geometric landmarks.

In a geometrically degenerate tunnel:
- Every wall looks identical
- LiDAR returns are uniform — no corners, edges, or features to lock onto
- There is nothing to "recognize" for localization
- GPS does not penetrate underground (GNSS-denied)
- The robot has **no reliable way to know where it is**

This is why GEODE was created — existing datasets did not have enough of these hard environments, so SLAM algorithms were never properly tested against them.

### Which GEODE Sequence Are You Using?
You are using the **tunnel sequence(s)** from GEODE — specifically underground tunnel data — to reconstruct a faithful simulation environment in MuJoCo. The geometry of your simulated tunnel is derived directly from real LiDAR scans, not hand-crafted.

---

## 2. Project Overview

### What You Have Built
| Component | Description |
|---|---|
| **Simulation Environment** | MuJoCo reconstruction of a real underground tunnel from GEODE LiDAR data |
| **Robot Platform** | GEODE dataset's collection platform (UGV / handheld — tunnel sequence) |
| **Environment Type** | Underground tunnel — geometrically degenerate, GNSS-denied |
| **Data Source** | Real LiDAR scans from the GEODE dataset |
| **Constraint** | Tunnel geometry is fixed — derived from real data, cannot be modified |

### What "Real2Sim" Means for Your Project
Most simulation papers go **Sim → Real** (train in sim, deploy in real world).

Your project does the opposite: **Real → Sim**

```
Real Tunnel (exists physically)
        ↓
LiDAR Scans collected (GEODE dataset)
        ↓
3D Geometry extracted from point cloud
        ↓
MuJoCo environment reconstructed
        ↓
Robot navigates in faithful simulation
```

This is your strongest differentiator. You are not designing challenges — you are **preserving** challenges that already exist in the real world.

### Why GNSS-Denial is Not an Assumption — It Is a Fact
Underground tunnels do not receive GPS signals. This is a physical constraint, not a modeling choice. You do not need to justify or argue for GNSS-denial in your paper — it is simply true of the environment you are simulating. This is important: reviewers cannot challenge this as an artificial setup.

---

## 3. Workshop Selection & Justification

### Selected Workshop
**Workshop 31: Planning and Control with Imperfect Sensors and Perception**
- Date: Friday, July 17, 2026 (Afternoon)
- Submission: OpenReview — `https://openreview.net/group?id=roboticsfoundation.org/RSS/2026/Workshop/WPCIS`
- Format: 2-page extended abstract (excluding references), RSS LaTeX format
- Non-archival venue (can publish elsewhere after)

### Why This Workshop Over Others

**vs Workshop 13 (Sim2Real):**
Workshop 13 explicitly focuses on manipulation, deformable objects, contact dynamics, and tactile sensing. Underground tunnel navigation does not fit that scope. Workshop 31 explicitly mentions field robotics, noisy localization, partial observability, and POMDP — all directly relevant.

**vs Workshop 17 (Open-World Navigation):**
Workshop 17 leans toward foundation models and language-conditioned navigation. Your work is grounded, physics-based, and dataset-driven — a stronger fit for Workshop 31's rigorous uncertainty-handling focus.

### Invited Speakers — Why This Matters for You
| Speaker | Expertise | Relevance to Your Paper |
|---|---|---|
| **Hanna Kurniawati** (ANU) | World-leading POMDP researcher | Your GNSS-denied formulation is a POMDP — she is the audience |
| **Yiannis Kantaros** (WashU) | Formal methods, safety guarantees | Your safety-under-uncertainty angle appeals directly to him |
| **Rajat Talak** (NUS) | Perception-planning interface | Your Real2Sim environment study fits his interests |

### Workshop Topics Your Paper Directly Hits
- ✅ Planning and control under sensing uncertainty
- ✅ Partially Observable Markov Decision Processes (POMDP)
- ✅ Sim-to-real transfer and uncertainty quantification
- ✅ Active perception and information gathering (optional addition)
- ✅ Learned planning and control
- ✅ Benchmarks, datasets, and evaluation metrics (explicitly requested by organizers)

---

## 4. Core Paper Argument

### One Paragraph Summary
> Underground tunnels represent one of the most extreme real-world instances of planning under sensing absence. They are geometrically degenerate (featureless, uniform geometry), GNSS-denied (no absolute position), and present a partial observability challenge that synthetic environments cannot replicate faithfully. We present a MuJoCo simulation of a real underground tunnel, reconstructed directly from the GEODE LiDAR dataset, preserving the authentic geometric degeneracy of the real environment. We validate this reconstruction through three complementary methods and conduct a navigation study under GNSS-denied conditions to demonstrate that real-world-derived tunnel geometry creates planning challenges absent in synthetic benchmarks.

### The Core Claim
**"Our environment is not designed to be hard — it is hard because it is real."**

This single claim separates your paper from every other simulation-based navigation paper. You did not tune parameters to create difficulty. The difficulty was collected with a LiDAR sensor from a real tunnel.

### The Scientific Question Your Paper Answers
> *Does faithfully reconstructing a geometrically degenerate real tunnel in simulation preserve the planning challenges present in the real environment?*

**Your answer:** Yes — and here is quantitative evidence from three validation methods and a navigation study.

---

## 5. The Three Contributions

### Contribution 1: The Environment (Primary)
A MuJoCo underground tunnel environment reconstructed from real GEODE LiDAR data.

**What makes it a contribution:**
- First (or among first) MuJoCo environments derived from a geometrically degenerate real tunnel dataset
- Faithful geometry — not hand-crafted, not procedurally generated
- GNSS-denial is inherent, not artificially imposed
- Reproducible — others can use GEODE data and verify your reconstruction
- Openly available benchmark for the community

**How to describe it:**
The environment is not a contribution of novelty in design — it is a contribution of **fidelity and authenticity**. The value is precisely that you did not design it.

---

### Contribution 2: Three-Method Validation (Secondary)
A rigorous validation framework showing the MuJoCo reconstruction preserves real-world planning challenges.

This answers the implicit question every reviewer will have: *"How do we know your simulation is actually faithful to the real tunnel?"*

Three methods are used (detailed in Section 6).

---

### Contribution 3: Navigation Study (Scientific)
An empirical study of navigation performance under GNSS-denied conditions in the validated environment.

This transforms the paper from an environment paper into a **scientific study**. The environment is the testbed. The navigation study is the science.

---

## 6. Validation Strategy — All Three Methods

This section is the backbone of your paper's credibility. You are validating without ever deploying a physical robot — entirely legitimate because the real data already exists in GEODE.

---

### Validation Method 1: Geometric Faithfulness Check

**What it is:**
A quantitative comparison between your MuJoCo reconstruction and the original GEODE point cloud.

**How it works:**
1. Take the original GEODE tunnel LiDAR point cloud (you already have this — it's the source data)
2. Render a simulated LiDAR scan from your MuJoCo tunnel at the same positions as real GEODE scans
3. Compute numerical distance metrics between the two point clouds

**Metrics to use:**
- **Chamfer Distance** — average distance from each point in set A to its nearest neighbor in set B, and vice versa. Lower = more faithful reconstruction
- **Point-to-Plane Distance** — measures how well surface normals match, capturing wall orientation accuracy
- **Hausdorff Distance** — captures the worst-case mismatch between the two point clouds

**What this proves:**
Your simulation is geometrically faithful to the source data. The walls are in the right place, the corridor widths are accurate, the tunnel shape is preserved.

**What to show in the paper:**
- A table of Chamfer/Hausdorff distances per tunnel section
- A side-by-side visualization of real point cloud vs simulated point cloud
- Qualitative images showing the reconstruction looks like the real tunnel

**Tools needed:**
- Open3D (Python) for point cloud processing and metric computation
- MuJoCo's built-in rendering for simulated scan generation

---

### Validation Method 2: SLAM Algorithm Replay

**What it is:**
Running an established, published SLAM algorithm on both the real GEODE data and your MuJoCo environment, then comparing the results.

**How it works:**
1. Select a standard LiDAR SLAM algorithm (e.g., LIO-SAM, LOAM, FastLIO — all published, all available on GitHub)
2. Run it on the **real GEODE tunnel sequence** → Record trajectory and localization error vs ground truth
3. Run the **exact same algorithm** in your MuJoCo environment (simulate the same sensors, same trajectory) → Record trajectory and localization error
4. Compare: does the algorithm fail in the same places? Does it succeed in the same places?

**What this proves:**
If a real algorithm behaves similarly in your simulation as it does on real data — your simulation is not just geometrically faithful, it is **behaviorally faithful**. The hard spots are hard for the same reasons.

**GEODE gives you ground truth for free:**
GEODE provides ground truth trajectories for every sequence. This means you know exactly where the algorithm should be at every point. You can compute RMSE (root mean square error) for both real and simulated runs and compare them directly.
(GEODE even provides `rmse.py` script for this — use it directly)

**What to show in the paper:**
- Trajectory plots: ground truth vs estimated, for real data and sim data side-by-side
- RMSE comparison table: real GEODE vs MuJoCo reconstruction
- Failure point analysis: where does localization diverge, and is it the same location in both?

**Key finding to aim for:**
The SLAM algorithm degrades at the same tunnel sections in both real data and simulation — proving that the degenerate geometry is faithfully reproduced.

---

### Validation Method 3: Degeneracy Preservation Check

**What it is:**
A quantitative measurement of geometric degeneracy along the tunnel path, comparing real GEODE data to your MuJoCo reconstruction.

**How it works:**
Geometric degeneracy can be measured using the **eigenvalue analysis of the LiDAR point cloud covariance matrix**. This is a standard technique in the SLAM community:

1. At each position along the tunnel, take a local LiDAR scan
2. Compute the covariance matrix of the point cloud
3. Compute the eigenvalues λ₁ ≥ λ₂ ≥ λ₃
4. The **degeneracy score** is related to the ratio of the smallest eigenvalue to the largest — when this ratio is very small, the environment is degenerate (tunnel-like, planar, featureless)
5. Plot this score along the full tunnel path

**Run this on:**
- Real GEODE tunnel LiDAR data → Degeneracy profile A
- Your MuJoCo reconstruction → Degeneracy profile B

**What this proves:**
If the degeneracy profiles match — your simulation preserves the exact locations and degrees of difficulty that make the real tunnel hard. A synthetic hand-crafted tunnel would not match this profile.

**What to show in the paper:**
- A graph: Degeneracy score vs distance along tunnel, for both real data (GEODE) and simulation
- Highlight the high-degeneracy zones (fully featureless sections) — show they appear at the same tunnel positions in both
- This is your most technically rigorous validation — it directly argues that the planning challenges are authentic

**Why this is your strongest argument:**
Any reviewer who says "but is this realistic?" gets this graph as the answer. The degeneracy signature of your simulation matches the degeneracy signature of the real tunnel, measured quantitatively.

---

### Summary: What the Three Validations Prove Together

| Validation | What It Checks | What It Proves |
|---|---|---|
| **Method 1: Geometric Faithfulness** | Shape and dimensions | The tunnel looks like the real tunnel |
| **Method 2: SLAM Algorithm Replay** | Algorithmic behavior | The tunnel is hard in the same ways as the real tunnel |
| **Method 3: Degeneracy Preservation** | Featurelessness profile | The tunnel preserves the exact planning challenges of the real tunnel |

Together: **"Our simulation is the real tunnel, inside MuJoCo."**

---

## 7. Navigation Study — The Scientific Experiment

After validating the environment, you run a navigation study to show what planning looks like in a real geometrically degenerate, GNSS-denied tunnel.

### Setup
- Robot navigates from start to end of the tunnel
- **No GPS / global position available** (GNSS-denied — inherent to underground)
- Robot only has: relative odometry + onboard LiDAR/depth observations
- Add **configurable odometry drift noise** to simulate realistic sensor degradation

### Three Conditions to Compare

| Condition | Observation Space | What It Represents |
|---|---|---|
| **Oracle (Baseline)** | Full ground truth pose from MuJoCo | Upper bound — perfect localization |
| **GNSS-Denied (Realistic)** | Relative odometry only, no global pose | Real operational constraint |
| **GNSS-Denied + Drift** | Odometry with cumulative Gaussian noise | Realistic sensor degradation over distance |

### What to Run in Each Condition

**Option A: RL Agent (Learned Planning)**
- Train a policy with observations matching each condition
- Proximal Policy Optimization (PPO) or SAC — standard, well-understood
- Observation space: strip global (x,y,z) from MuJoCo state, keep relative heading + depth returns
- Reward: progress toward goal minus collision penalty

**Option B: Classical Planner with Particle Filter (Belief-Space Planning)**
- Particle filter maintains a distribution over robot pose
- RRT or potential field planner operates on the belief state
- No GPS → belief uncertainty grows over time → planning becomes conservative

**Option C: Both (Strongest Paper)**
- Run RL and classical planner under all three conditions
- Compare: does learned planning handle uncertainty differently from explicit belief-space planning?
- This directly answers the workshop's core question: *"What are the trade-offs between modular pipelines and end-to-end learning?"*

### Metrics to Report

| Metric | Why It Matters |
|---|---|
| **Task success rate** | Did the robot reach the goal? |
| **Collision rate** | Safety under uncertainty |
| **Localization RMSE** | How far off is estimated pose from ground truth? |
| **Belief entropy over time** | How fast does uncertainty grow without GPS? |
| **Performance vs noise level** | At what point does each method fail? |

### Key Finding to Aim For
A **phase transition**: below a certain noise level, all methods work. Above it, only uncertainty-aware methods survive. This is a clean, publishable scientific finding that directly answers workshop questions about when and why uncertainty handling is non-negotiable.

---

## 8. Full Paper Structure (2-Page Abstract)

### Title (Options)
1. *"Grounded in Reality: A MuJoCo Benchmark for Navigation Under GNSS-Denial from Real Tunnel LiDAR Data"*
2. *"From GEODE to Simulator: A Faithful Reconstruction of Geometrically Degenerate Tunnels for Planning Under Sensing Absence"*
3. *"Real Tunnels, Real Challenges: A MuJoCo Benchmark for GNSS-Denied Navigation from the GEODE Dataset"*

---

### Paragraph 1 — Motivation and Problem (~100 words)
**What to say:**
- Autonomous robots are increasingly deployed in GNSS-denied underground environments
- Tunnels are geometrically degenerate — no landmarks, featureless walls, no absolute localization possible
- Existing simulation environments are synthetic and do not capture real tunnel geometry
- This creates a gap: algorithms are tested on idealized simulations but must operate in real tunnels
- We address this gap

**Key phrases to use:**
- "geometrically degenerate"
- "GNSS-denied"
- "partial observability"
- "sensing absence"

---

### Paragraph 2 — Environment Contribution (~120 words)
**What to say:**
- We present a MuJoCo reconstruction of a real underground tunnel from the GEODE dataset
- GEODE provides real LiDAR scans with ground truth trajectories — we use the tunnel sequence
- Our reconstruction preserves real geometry: corridor dimensions, wall profiles, featureless sections
- GNSS-denial is inherent — not artificially imposed — because the environment is underground
- The environment is a POMDP: state is full robot pose, observations are partial (no GPS), transition follows MuJoCo physics

**Key phrases to use:**
- "Real2Sim reconstruction"
- "GEODE tunnel sequence"
- "authentic geometric degeneracy"
- "formally specified as a POMDP"

---

### Paragraph 3 — Three-Method Validation (~150 words)
**What to say:**
- We validate fidelity through three complementary methods
- Method 1: Geometric faithfulness — Chamfer distance between real point cloud and simulated scans
- Method 2: SLAM algorithm replay — same algorithm run on real GEODE data and our simulation, behavior compared
- Method 3: Degeneracy preservation — eigenvalue-based degeneracy profile matched against real tunnel data
- Results show our reconstruction preserves the location and magnitude of degeneracy present in the real environment
- This distinguishes our environment from synthetic benchmarks: the challenges are real, not designed

---

### Paragraph 4 — Navigation Study (~150 words)
**What to say:**
- We deploy [RL / classical planner / both] in the validated environment under three localization conditions: oracle, GNSS-denied, GNSS-denied with drift
- Report success rate, collision rate, localization RMSE, and belief entropy across conditions
- Key finding: [your result — aim for phase transition finding]
- Real tunnel geometry creates failure modes absent in synthetic environments: dead-reckoning divergence in long straight sections, belief multimodality at symmetric junctions
- These findings are only possible because the environment is derived from real geometry

---

### Paragraph 5 — Takeaway and Workshop Connection (~80 words)
**What to say:**
- Our benchmark provides a reproducible testbed grounded in real-world geometry for the community
- It directly addresses the workshop's call for new benchmarks measuring reliability in real-world deployment
- Findings characterize when and why explicit uncertainty handling is necessary — not on a synthetic test, but on a real tunnel

---

## 9. Task Breakdown & Timeline

Deadline: **June 5, 2026** — approximately 10 days from project start.

| Task | Details | Estimated Effort |
|---|---|---|
| **GEODE tunnel data preparation** | Download tunnel sequence, extract point cloud, identify ground truth trajectory | 0.5 days |
| **MuJoCo environment finalization** | Confirm tunnel mesh is correct, set up simulation parameters, confirm GEODE robot dimensions match | 1 day |
| **Validation Method 1** | Simulate LiDAR scans from MuJoCo, compute Chamfer + Hausdorff distances vs GEODE point cloud using Open3D | 1.5 days |
| **Validation Method 2** | Run LIO-SAM or FastLIO on real GEODE data, run same on MuJoCo (using ROS bridge or direct sim), compare RMSE using GEODE's rmse.py | 2 days |
| **Validation Method 3** | Compute eigenvalue-based degeneracy profile along tunnel path for both real and sim data, plot comparison | 1 day |
| **Navigation study setup** | Strip GPS from observation space, add odometry drift noise, set up reward function | 1 day |
| **Run experiments** | Oracle, GNSS-denied, GNSS-denied+drift conditions — run to convergence, collect metrics | 1.5 days |
| **Figures and plots** | Point cloud comparison, trajectory plots, degeneracy profiles, performance vs noise curves | 1 day |
| **Write 2-page abstract** | RSS LaTeX format, all five paragraphs | 1.5 days |
| **Buffer / revisions** | Unexpected issues, refinement | 1 day |
| **TOTAL** | | **~12 days** |

---

## 10. Key Terminology to Use in the Paper

Use these terms deliberately and consistently — they are the vocabulary of Workshop 31 and signal to reviewers that your work belongs there.

| Term | What It Means in Your Paper |
|---|---|
| **Geometric degeneracy** | Tunnel walls are featureless — no landmarks for LiDAR to lock onto |
| **GNSS-denied** | No GPS underground — absolute position is unavailable |
| **Partial observability** | The robot cannot observe its full state (no global pose) |
| **POMDP** | Formal way to frame your navigation task: state, observation, action, uncertainty |
| **Belief state** | The robot's probability distribution over where it might be, given noisy observations |
| **Localization drift** | Accumulated odometry error that grows over distance without GPS correction |
| **Real2Sim** | The process of reconstructing a real environment in simulation |
| **Planning under sensing absence** | Stronger than "imperfect sensing" — GPS is not noisy, it simply does not exist underground |
| **Phase transition** | The noise threshold above which planning methods fail — your key experimental finding |
| **Degeneracy profile** | Quantitative measure of featurelessness along the tunnel path |

---

## 11. Arguments Against Common Review Criticisms

Anticipate reviewer pushback and pre-empt it in the paper.

---

**Criticism: "This is just an environment paper — where is the novelty in the method?"**

**Response:** Workshop 31 explicitly requests new benchmarks and asks *"What benchmarks and metrics are needed to measure reliability in real-world deployment?"* Our contribution is a validated, real-data-derived benchmark — this is a direct answer to the organizers' stated need. The navigation study provides scientific findings, not just an environment demo.

---

**Criticism: "Your simulation may not reflect the real tunnel accurately."**

**Response:** Three validation methods demonstrate fidelity at the geometric level (Chamfer distance), behavioral level (SLAM algorithm replay), and challenge-preservation level (degeneracy profile matching). No other simulation paper in this domain offers this level of validation rigor.

---

**Criticism: "Why not just use the GEODE data directly instead of building a simulation?"**

**Response:** GEODE provides fixed recorded trajectories. A MuJoCo simulation allows interactive navigation, policy training, arbitrary starting positions, and controllable noise injection — none of which are possible with a fixed dataset. The simulation enables the planning study that the dataset alone cannot.

---

**Criticism: "GNSS-denial is trivially addressed by dead-reckoning."**

**Response:** In geometrically degenerate tunnels, dead-reckoning degrades faster than in standard environments because there are no features to periodically correct the estimate. Our results show that real tunnel geometry creates localization drift patterns that are qualitatively different from synthetic corridors — this is precisely what our degeneracy preservation validation demonstrates.

---

**Criticism: "Why MuJoCo and not a more robotics-focused simulator?"**

**Response:** MuJoCo provides accurate physics for ground vehicle dynamics, is widely used in the RL community, and enables direct comparison with a large body of learned planning work. Our environment is available as a standard MuJoCo XML, lowering the barrier for the community to use it.

---

## Summary — What This Paper Is

| Aspect | Description |
|---|---|
| **Core contribution** | First MuJoCo benchmark of a real geometrically degenerate tunnel from GEODE LiDAR data |
| **Key differentiator** | Real data source — challenges are authentic, not designed |
| **Validation** | Three-method: geometric fidelity + SLAM replay + degeneracy preservation |
| **Scientific study** | Navigation performance under GNSS-denial and localization uncertainty |
| **Workshop fit** | Planning under sensing absence, POMDP, benchmarks — all explicitly requested topics |
| **Audience appeal** | Directly relevant to all three invited speakers |
| **Format** | 2-page extended abstract, RSS LaTeX, non-archival |
| **Deadline** | June 5, 2026 |
| **Submission link** | `https://openreview.net/group?id=roboticsfoundation.org/RSS/2026/Workshop/WPCIS` |

---

*Document prepared for RSS 2026 Workshop 31 submission planning.*
*All workshop details sourced from official RSS 2026 program and workshop websites.*
