"""Read-only, strict pre-START R-B DDS state-path evidence validation.

This function never makes an actuator decision. A failed observation blocks
sending START from the Solo capture script, NOT the Guard/Simulator logic.
"""


def evaluate_rb_state_path(subscriptions, publishers, diagnostics):
    """Return (admissible, detailed evidence) from actual ROS endpoint facts.

    Endpoints are normalized dicts: name, reliability, durability, depth.
    The producer must be the simulator's RELIABLE state writer. Fast DDS
    can report endpoint depth 0/UNKNOWN even for the verified source's 16.
    We check known depth strictly; provenance hashes pin the source setting.
    """
    rb = [x for x in subscriptions if x.get('name') == 'r_b0_racing_pursuit']
    producer = [x for x in publishers if
                x.get('name') == 'competition_simulator0']
    endpoint_ok = (len(rb) == 1 and
                   rb[0].get('reliability') == 'BEST_EFFORT' and
                   rb[0].get('depth') in (0, 1) and
                   rb[0].get('durability') == 'VOLATILE' and
                   len(producer) == 1 and
                   producer[0].get('reliability') == 'RELIABLE' and
                   (producer[0].get('depth') == 0 or
                    producer[0].get('depth', -1) >= 16) and
                   producer[0].get('durability') == 'VOLATILE')
    facts = {
        'rb_state_endpoints': rb,
        'sim_state_publishers': producer,
        'rb_qos_endpoint_verified': bool(endpoint_ok),
        'dds_depth_reporting': ('UNKNOWN_BY_MIDDLEWARE' if
            any(e.get('depth') == 0 for e in rb+producer) else 'OBSERVED'),
    }
    required = ('state_rx_count', 'state_rx_last_sequence',
                'state_rx_recent_ages_ns',
                'state_rx_last_receipt_steady_age_ns',
                'state_rx_recent_8_fresh')
    if not isinstance(diagnostics, dict) or not all(k in diagnostics for k in required):
        facts['rb_state_diagnostics'] = 'MISSING_OR_UNTRUSTED'
        facts['pass'] = False
        return False, facts
    ages = diagnostics['state_rx_recent_ages_ns']
    sequence = diagnostics['state_rx_last_sequence']
    receipt_age_ns = diagnostics['state_rx_last_receipt_steady_age_ns']
    status_ok = (
        diagnostics.get('schema_version') == 2 and
        diagnostics.get('type') == 'R_B_CONTROL_WORKER_SCHEDULING' and
        diagnostics.get('state_rx_qos_reliability') == 'BEST_EFFORT' and
        diagnostics.get('state_rx_qos_depth') == 1 and
        type(diagnostics['state_rx_count']) is int and
        diagnostics['state_rx_count'] >= 8 and
        type(sequence) is int and sequence >= 7 and
        isinstance(ages, list) and len(ages) == 8 and
        all(type(v) is int and 0 <= v <= 75_000_000 for v in ages) and
        type(receipt_age_ns) is int and 0 <= receipt_age_ns <= 250_000_000 and
        diagnostics['state_rx_recent_8_fresh'] is True)
    facts['rb_state_diagnostics'] = {k: diagnostics[k] for k in required}
    facts['rb_state_diagnostics_fresh'] = bool(status_ok)
    facts['pass'] = bool(endpoint_ok and status_ok)
    return facts['pass'], facts
