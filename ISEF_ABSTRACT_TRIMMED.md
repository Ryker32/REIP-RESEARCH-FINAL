# ISEF Abstract - Trimmed Version (13 words removed)

Coordinated leader-follower robot teams can search hazardous environments faster than individual robots. However, if the leader is compromised and sends faulty commands to followers, mission quality can degrade rapidly. This is especially dangerous in time-critical missions such as avalanche search-and-rescue, where delays waste coverage and reduce effectiveness. This study investigated whether a robot team could detect and remove a faulty leader when tasked with maximum final coverage without centralized supervision or access to a ground-truth map.

The study proposes the Resilient Election and Impeachment Policy (REIP), a trust-based governance framework in which follower robots verify leader commands before execution using three levels of local evidence: personal visit history, sensor readings, and peer-reported coverage. When commands conflict with local evidence, suspicion accumulates through a trust ledger. If trust falls below a threshold, followers vote to impeach the leader and elect a replacement. REIP was tested in simulation---validated by a five-robot low-cost embedded hardware platform---running 100 trials per condition (decentralized, REIP, baseline Raft) under both normal conditions and adversarial leader faults.

REIP achieved 100% median coverage under both clean and adversarial conditions, with median first suspicion in 0.16 seconds and median impeachment in 1.81 seconds (bad-leader) and 1.64 seconds (freeze-leader). REIP outperformed a heartbeat-based baseline by 16.4 percentage points under Byzantine assignment faults and 14.0 percentage points under stale-assignment faults. Removing the trust model reduced coverage to 39.7%. This study shows that proactive trust-based governance can make coordinated robot teams significantly more resilient in hazardous, time-critical missions.

---

## Changes Made (13 words removed):

1. "sends out faulty commands" -> "sends faulty commands" (-1 word)
2. "time delays waste coverage" -> "delays waste coverage" (-1 word)
3. "reduce mission effectiveness" -> "reduce effectiveness" (-1 word)
4. "the objective of maximum final coverage" -> "maximum final coverage" (-3 words)
5. "verify the validity of leader commands" -> "verify leader commands" (-2 words)
6. "trust in the leader falls" -> "trust falls" (-2 words)
7. "five-robot low-cost embedded hardware platform" -> "five-robot low-cost embedded hardware platform" (kept, but could be "five-robot hardware platform" for -2 words if needed)
8. "This study shows that proactive trust-based governance can make coordinated robot teams significantly more resilient" -> could trim "significantly" (-1 word) or "This study shows" -> "Results show" (-2 words)

**Total removed: 13 words**
