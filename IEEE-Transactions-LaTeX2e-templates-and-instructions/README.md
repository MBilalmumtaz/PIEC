# PIEC TEVC Paper — IEEE Transactions on Evolutionary Computation

This directory contains the LaTeX source for the TEVC manuscript:

> **Real-Time Warm-Start NSGA-II with Surrogate-Assisted Evaluation for
> Seven-Objective Energy-Aware Path Planning in Autonomous Ground Vehicles**

---

## Directory Contents

| File / Folder | Description |
|---|---|
| `piec_tevc_paper.tex` | **Main manuscript** (start here) |
| `piec_tevc_references.bib` | BibTeX bibliography (reused from Springer Nature companion paper) |
| `IEEEtran.cls` | IEEE journal LaTeX class file (do not modify) |
| `figures/` | Figures directory (see `figures/README_FIGURES.txt` for required figures) |
| `bare_jrnl_new_sample4.tex` | Original IEEEtran sample file (reference only) |

---

## How to Compile

### Option 1 — `pdflatex` + `bibtex` (recommended)

```bash
cd IEEE-Transactions-LaTeX2e-templates-and-instructions

pdflatex piec_tevc_paper
bibtex   piec_tevc_paper
pdflatex piec_tevc_paper
pdflatex piec_tevc_paper
```

Run `pdflatex` **three times** after `bibtex` to resolve all
cross-references and bibliography entries.

### Option 2 — `latexmk` (single command)

```bash
cd IEEE-Transactions-LaTeX2e-templates-and-instructions

latexmk -pdf -bibtex piec_tevc_paper
```

`latexmk` automatically runs the necessary passes.

### Option 3 — `latexmk` with continuous rebuild

```bash
latexmk -pdf -bibtex -pvc piec_tevc_paper
```

Watches for file changes and recompiles automatically (useful during editing).

---

## Required LaTeX Packages

The following packages must be available in your TeX distribution
(all are included in TeX Live and MiKTeX by default):

- `amsmath`, `amssymb`, `amsfonts`
- `algorithmic`, `algorithm`
- `array`, `booktabs`, `multirow`
- `graphicx`
- `cite`
- `textcomp`, `stfloats`
- `url`
- `xcolor`
- `subfig`

---

## Adding Figures

Place figure files (PDF or PNG/EPS) in the `figures/` subdirectory.
See `figures/README_FIGURES.txt` for a description of each required figure.

Example inclusion in the paper:
```latex
\begin{figure}[t]
  \centering
  \includegraphics[width=\columnwidth]{figures/fig_system}
  \caption{RT-NSGA-II system architecture.}
  \label{fig:system}
\end{figure}
```

---

## Paper Structure

| Section | Content |
|---|---|
| 1. Introduction | EC motivation, bottleneck identification, contributions |
| 2. Problem Formulation | 7-objective UGV path planning; Pareto incompatibility theorem |
| 3. Proposed Algorithm (RT-NSGA-II) | Warm-start; surrogate evaluation; adaptive mutation; complexity |
| 4. Experimental Setup | Scout Mini hardware; simulation and real-world environments; baselines |
| 5. Results | Algorithm performance (hypervolume); navigation; real-world; ablation |
| 6. Discussion | EC insights: warm-start conditions, surrogate accuracy threshold |
| 7. Related Work | MOEA literature; surrogate-assisted EA; evolutionary path planning |
| 8. Conclusion & Future Work | Summary; generalization; open problems |
| Appendix A | Full proof of Theorem 2 (surrogate advantage condition) |
| Appendix B | UKF state estimator summary |

---

## Key Constraints

- **No PINN** anywhere in this manuscript.
- **No DWA** anywhere in this manuscript.
- References are reused from the companion Springer Nature paper (`piec_references.bib`);
  no new citations have been invented.
- The `springer-nature-article/` directory was **not modified**.

---

## Submission Notes

- Target venue: **IEEE Transactions on Evolutionary Computation (TEVC)**
- Document class: `IEEEtran` (`lettersize`, `journal`)
- Template version: `IEEEtran.cls` as provided in this directory
- Expected length: 12–15 pages in double-column IEEE format
