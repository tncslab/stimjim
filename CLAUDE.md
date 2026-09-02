# Python environment

This project uses the conda environment `compute` or a python venv `fusenv`, depending on the machine.
It is NOT auto-activated, and the bash tool runs in a non-interactive shell, so `conda activate` alone
will fail. Use these patterns instead.

- Run Python and tools through the env without activating:
    conda run -n compute python <script>.py
    conda run -n compute pytest
    conda run -n compute pip list

- If you genuinely need an activated interactive session, source conda first:
    source "$(conda info --base)/etc/profile.d/conda.sh" && conda activate compute

- Never use the base/system Python.
- Confirm the interpreter with: conda run -n compute which python


# Coding style

- Whenever possible use established python scientific comuting libraries, such as `numpy`, `scipy`, `numba`, `pandas`, `scikit-learn`, `pytorch`, etc.
- This project a research tool, **do not implement unnecessary conversions or branches taht enforce default values**. Instead, assume that input is in the proper format and array shape and arguments are always passed when necessary. If something goes wrong it shall trigger an Exception either by manual assert or in the invoked libraries.
- Some functions may use nontrivial indexing or advanced python features, if their deciphering requires effort, **leave explaining comments between the lines**.
- Ask for **explicit confirmation before removing** manually commented out or unreachadble code sections. If not asked for cleanup, leave commented out sections and unused branches.
- **Suggest variable name changes** but ask for confirmation if it breaks backward compatibility. Indications for change: new name would better describe semantic meaning than the previous ones, previous name is misunderstood, new names unify naming convention (capital letters, order of properties).
- When running speed or prerformance **benchmarks in the foreground, make sure they have a timeout** of 3 minutes top to prevent blocking. You may retry with smaller batch or defer tests to user approval.

# Phased projects

When beginning a new phase, make sure that changes from the previous phase are commited to the git repo if it exists.

At the end of each phase, write the handoff entry — decisions, rationale, open questions, next
entry point — to `docs/progress/<NNN>-<topic>.md` and prepend its one-line summary to the index
in @docs/PROGRESS.md (newest first). To pick up work, read the index and then at most the newest
one or two entry files, never the whole log. If all objectives are fulfilled, commit to the
existing git repo; then stop.

Development story and historic references live exclusively in the progress reports and git messages.
All other documents describe current state, rationale, known limitaions and remaining tasks without referring to history.

# GPU notes (this project: OpenMM MD)

- Development hardware: **GeForce GTX 950** (2GB, Maxwell cc 5.2, no `COOPERATIVE_LAUNCH`)
  and **GeForce MX450** (2 GB, cc 7.5), small test systems only.
- Deploy hardware: **RTX 3070 Ti** (8 GB, cc 8.6) — the main target.
- Use **mixed (FP32) precision** — default and ideal for consumer cards; weak FP64 is
  irrelevant to OpenMM MD throughput. **VRAM**, not FP64, bounds system size.
- OpenMM compiles kernels at runtime via NVRTC, so the conda `cuda-version` must not
  exceed the driver's max CUDA, or you get `CUDA_ERROR_UNSUPPORTED_PTX_VERSION`. Reconcile
  by updating the driver (what was done here: driver 610.47 / CUDA 13.3) or by pinning an
  older-CUDA OpenMM build.

# Pressure Field Measurement System

## Instruments

- **Function Generator**: Rigol DG800 Pro — manual at `docs/manuals/rigol_dg800pro.pdf` and `docs/manuals/rigol_dg800pro_programming.pdf`
- **Oscilloscope**: Tektronix TDS 2004B — manual at `docs/manuals/tektronix_tds2004b.pdf` and `docs/manuals/tektronix_tds2000_programmer.pdf`
- **Scanner**: Creality Ender V3 SE — manual at `docs/manuals/creality_ender_v3_se.pdf`

## Setup

Run `python scripts/download_manuals.py` before first use to fetch manuals.



