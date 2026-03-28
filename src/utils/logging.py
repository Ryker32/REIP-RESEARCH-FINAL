import csv, os

class RunLogger:
    def __init__(self, cfg, run_name="default"):
        self.run_dir = os.path.join("runs", run_name)
        os.makedirs(self.run_dir, exist_ok=True)

        self.fp = open(os.path.join(self.run_dir, "log.csv"), "w", newline="")
        self.w = csv.writer(self.fp)
        
        # Enhanced headers for new metrics
        headers = [
            "t", "coverage", "leader", "elections",
            # Enhanced REIP metrics
            "impeachments", "loop_detections", "aimless_detections",
            "avg_trust", "demoted_leaders",
            # Trust and hallucination diagnostics
            "predicted_gain", "observed_gain", "hallucinating",
            "hallucination_type", "origin_leader", "fallback_blocked",
            # Graph health and adaptive connectivity telemetry
            "graph_avg_degree", "graph_component_count", "graph_ttl_risk_frac", "cohesion_scale",
            "eff_connectivity_beta", "eff_tether_lambda", "eff_degree_under", "eff_spacing",
            # Fallback/autonomy actions
            "fallback_applied",
        ]
        
        self.w.writerow(headers)

    def record(self, env, reip, t):
        # Basic metrics
        basic_data = [t, env.coverage(), reip.leader_id, reip.election_count]
        
        # Enhanced metrics (if available)
        enhanced_data = []
        
        # Impeachment count
        if hasattr(reip, 'impeachment_history'):
            enhanced_data.append(len(reip.impeachment_history))
        else:
            enhanced_data.append(0)
        
        # Loop detections
        if hasattr(reip, 'loop_detections'):
            enhanced_data.append(len(reip.loop_detections))
        else:
            enhanced_data.append(0)
        
        # Aimless detections
        if hasattr(reip, 'aimless_detections'):
            enhanced_data.append(len(reip.aimless_detections))
        else:
            enhanced_data.append(0)
        
        # Average trust in current leader
        if hasattr(reip, 'get_average_trust_in_leader'):
            try:
                enhanced_data.append(reip.get_average_trust_in_leader())
            except:
                enhanced_data.append(1.0)  # Default trust
        else:
            enhanced_data.append(1.0)
        
        # Number of demoted leaders
        if hasattr(reip, 'demoted_leaders'):
            enhanced_data.append(len(reip.demoted_leaders))
        else:
            enhanced_data.append(0)
        
        # Trust/hallucination diagnostics
        diag = ["", "", "", "", "", ""]
        if hasattr(reip, 'trust_history') and reip.trust_history:
            last = reip.trust_history[-1]
            diag = [
                last.get('predicted_gain', ''),
                last.get('observed_gain', ''),
                int(bool(last.get('hallucinating', False))),
                last.get('hallucination_type', ''),
                last.get('origin_leader', ''),
                int(bool(last.get('fallback_blocked', False))),
            ]

        # Graph/adaptive telemetry (safe defaults if unavailable)
        telem = [
            getattr(reip, 'last_avg_degree', ''),
            getattr(reip, 'last_component_count', ''),
            getattr(reip, 'last_ttl_risk_frac', ''),
            getattr(reip, 'last_connectivity_scale', ''),
            getattr(reip, 'last_eff_connectivity_beta', ''),
            getattr(reip, 'last_eff_tether_lambda', ''),
            getattr(reip, 'last_eff_degree_under', ''),
            getattr(reip, 'last_eff_spacing', ''),
            getattr(reip, 'last_fallback_applied', 0),
        ]

        # Write all data
        self.w.writerow(basic_data + enhanced_data + diag + telem)

    def flush(self):
        self.fp.close()
