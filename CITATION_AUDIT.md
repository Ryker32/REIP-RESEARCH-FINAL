# Citation Audit - Uncited Claims

## Abstract (Line 69)

**UNCITED:**
- "Raft, which governs production systems such as etcd and Kubernetes" - This is a factual claim about Raft's deployment. Should cite the Raft paper or a survey of production systems using Raft.

**FIX:** Add citation: "Raft, which governs production systems such as etcd and Kubernetes~\cite{ongaro2014raft},"

---

## Introduction - Motivation and Context (Lines 83-87)

**All claims appear properly cited ✓**

---

## Introduction - Problem Statement (Lines 91-95)

**UNCITED:**
- "A compromised leader does not need to crash to cause mission failure; it only needs to remain active while issuing low-quality or misleading assignments." - This is a general statement about leader-follower systems. Could cite a paper on leader-follower vulnerabilities, but it's also reasonable as a general observation.

**UNCITED:**
- "In search-and-rescue missions, avoidable time delay and error is unacceptable." - General statement, probably fine without citation.

---

## Introduction - Proposed Solution (Lines 99-101)

**UNCITED:**
- "Preliminary experimentation of REIP using a CUSUM--Bayesian fault detection~\cite{page1954cusum, gelman2013bayesian} found that reactive fault tolerance models only provide a partial solution because they attempt to remove faulty leaders after damage has already successfully occurred to a mission objective. This introduces detection latency that wastes physical time, battery, and coverage \cite{chandola2009anomaly}."

**ISSUE:** The claim "reactive fault tolerance models only provide a partial solution" and "This introduces detection latency" should cite papers that discuss reactive fault tolerance in multi-robot systems, not just anomaly detection surveys.

**FIX:** Either:
1. Remove the claim about reactive models being partial (make it your contribution)
2. Add citations to papers that discuss reactive fault tolerance limitations in multi-robot systems

---

## Background and Related Work (Lines 123-135)

**UNCITED:**
- "However, these assumptions are often difficult to satisfy in range-limited robot teams." - General statement, probably fine.

**UNCITED:**
- "This distinction is critical: a leader that is alive and communicating but issuing degraded commands (e.g., from sensor hallucination or adversarial compromise) will never trigger Raft's timeout-based detection, leaving the team under indefinite faulty leadership." - This is your analysis of Raft's limitations. Fine as-is since you're explaining why Raft doesn't work for your use case.

**UNCITED:**
- "There is no mechanism for detecting when the leader's directions become counterproductive, nor for replacing the leader without external supervision." - This is a gap statement. Could cite papers that show this gap, but it's reasonable as a general observation.

---

## Methodology Section

**All methodology appears to be your original work - no citations needed ✓**

---

## Experimental Design (Lines 459-565)

**UNCITED:**
- "The leader uses greedy nearest-frontier allocation~\cite{burgard2000collaborative}: robots are sorted by distance to their nearest frontier cell (preventing systematic ID bias), and each is assigned the nearest unassigned frontier with \emph{assignment persistence}---a robot retains its target until the cell is explored, preventing assignment thrashing that would cause inefficient zigzag movement."

**ISSUE:** "assignment persistence" appears to be your addition/modification. The citation to burgard2000collaborative is for greedy nearest-frontier, but "assignment persistence" might need clarification that it's your contribution.

**UNCITED:**
- "Localization is provided by an overhead camera (Logitech C922 at 1080p, 30\,fps) running ArUco marker detection~\cite{garrido2014aruco} on a separate PC." - Properly cited ✓

**UNCITED:**
- "Raft is the dominant consensus protocol in production distributed systems" (Line 583) - Should cite a survey or the Raft paper with context about production use.

**FIX:** "Raft is the dominant consensus protocol in production distributed systems~\cite{ongaro2014raft}"

---

## Results Section

**All results are your own - no citations needed ✓**

---

## Discussion Section

**UNCITED:**
- "Reactive detectors require $O(1/\epsilon^2)$ observations to distinguish a faulty leader from noise" - This is a general statement about statistical detection. Should cite a paper on statistical detection theory or hypothesis testing.

**FIX:** Add citation to statistical detection theory, e.g., a signal detection theory paper or hypothesis testing reference.

**UNCITED:**
- "Decentralized exploration avoids the single-point-of-failure problem but sacrifices coordination: agents operating on partial information inevitably duplicate coverage." - This is a general statement about decentralized systems. Could cite papers on decentralized exploration, but it's also reasonable as a general observation.

**UNCITED:**
- "A leader with a merged global map assigns non-overlapping frontiers and absorbs the cost of global map fusion, keeping per-agent compute lightweight on resource-constrained platforms." - This is a general statement about leader-follower benefits. Could cite simmons2000coordination or burgard2000collaborative, but it's reasonable as-is.

---

## Conclusion

**All claims appear to be your results or general statements - no citations needed ✓**

---

## Summary of Required Fixes

1. **Abstract Line 69:** Add citation for "Raft, which governs production systems such as etcd and Kubernetes"
2. **Line 583:** Add citation for "Raft is the dominant consensus protocol in production distributed systems"
3. **Discussion Section:** Consider citing statistical detection theory for the $O(1/\epsilon^2)$ claim, or remove the specific complexity claim

## Optional Improvements

- Line 101: The claim about reactive fault tolerance could be strengthened with citations to papers that discuss reactive fault tolerance in multi-robot systems
- Discussion: The decentralized vs. leader-follower tradeoff could cite relevant papers, though it's reasonable as general knowledge
