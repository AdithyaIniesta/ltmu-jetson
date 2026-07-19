# Ground station GUI patch — add the LTMU engine profile

Add to the ground station script (the one with `ENGINE_PROFILES`):

```python
ENGINE_LTMU = 3
ENGINE_NAMES[ENGINE_LTMU] = "ltmu"

ENGINE_PROFILES[ENGINE_LTMU] = {
    "params": {
        "VERIFIER_LOST_THRESH":   16,
        "VERIFIER_UPDATE_THRESH": 17,
        "REDETECT_ACCEPT_SCORE":  18,
        "REDETECT_INTERVAL":      19,
        "MOTION_SIGMA":           20,
        "NEG_BANK_MAX":           21,
        "NEG_MARGIN_WEIGHT":      22,
        "ENABLE_MOTION_PRIOR":    23,
    },
    "rows": [
        (16, "verifier_lost_thresh",   "Verifier Lost Thresh"),
        (17, "verifier_update_thresh", "Verifier Update Thresh"),
        (18, "redetect_accept_score",  "Redetect Accept Score"),
        (19, "redetect_interval",      "Redetect Interval"),
        (20, "motion_sigma",           "Motion Sigma (px)"),
        (21, "neg_bank_max",           "Neg Bank Max"),
        (22, "neg_margin_weight",      "Neg Margin Weight"),
    ],
    "toggles": [("ENABLE_MOTION_PRIOR", 23, "Motion Prior")],
    "max_id": 23,
}
```

Also add matching entries to `TELEM_TO_PARAM` so the read-only telemetry
sync resolves these ids (mirrors the existing `dnn_verify_interval` /
`dnn_veto_threshold` / `dnn_accept_threshold` rows — LTMU reuses those
same telemetry fields, see `docs/protocol.md`).

No other GUI changes needed — `engine_from_reserved()` already decodes
`reserved` and calls `_apply_engine()`, which will pick this profile up
automatically the first time an LTMU tracker's telemetry arrives.
