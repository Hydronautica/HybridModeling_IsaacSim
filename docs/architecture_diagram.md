# Architecture sketch

```
Floating Body (USD) -> Mooring Model (Python) -> Hybrid Controller -> Franka Torques
        ^                                                            |
        |____________________________________________________________|
```

Each block corresponds to a module in `extensions/` and is driven by `launch/run_hybrid.py`.
