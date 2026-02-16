# PhD Thesis: Physics-Informed Neural Networks for Energy-Efficient Control (PIEC)

**Author:** [Your Name]  
**Institution:** [Your University]  
**Department:** [Department of Computer Science / Robotics / Engineering]  
**Degree:** Doctor of Philosophy (Ph.D.)  
**Year:** 2024

## Overview

This repository contains the complete LaTeX source code for the doctoral thesis titled **"Physics-Informed Neural Networks for Energy-Efficient Control of Mobile Robots"**. The thesis presents a novel framework (PIEC) that integrates physics-informed neural networks with multi-objective optimization, Unscented Kalman Filtering, and Dynamic Window Approach for energy-efficient autonomous navigation.

## Thesis Structure

```
thesis/
├── main.tex                    # Main thesis document
├── abstract.tex                # Abstract
├── bibliography.bib            # BibTeX bibliography (100+ entries)
├── README.md                   # This file
│
├── chapters/                   # Main thesis chapters
│   ├── chapter1_introduction.tex
│   ├── chapter2_literature.tex
│   ├── chapter3_methodology.tex
│   ├── chapter4_implementation.tex
│   ├── chapter5_experiments.tex
│   ├── chapter6_results.tex
│   └── chapter7_conclusion.tex
│
├── appendices/                 # Supplementary materials
│   ├── appendix_a_code.tex     # Code snippets (PINN, NSGA-II, UKF, DWA)
│   ├── appendix_b_params.tex   # Complete parameter tables
│   └── appendix_c_data.tex     # Extended experimental data
│
└── figures/                    # Figures and images
    └── .gitkeep                # Placeholder directory
```

## Compilation Instructions

### Prerequisites

Install a complete LaTeX distribution:
- **Linux:** `sudo apt-get install texlive-full`
- **macOS:** Install MacTeX from https://www.tug.org/mactex/
- **Windows:** Install MiKTeX from https://miktex.org/

### Required LaTeX Packages

The thesis uses the following packages (typically included in full LaTeX distributions):

- **Document class:** `report`
- **Typography:** `fontenc`, `inputenc`, `microtype`
- **Layout:** `geometry`, `setspace`, `fancyhdr`
- **Graphics:** `graphicx`, `tikz`, `pgfplots`, `subfig`
- **Tables:** `booktabs`, `array`, `multirow`, `longtable`
- **Math:** `amsmath`, `amssymb`, `amsthm`, `mathtools`
- **Algorithms:** `algorithm`, `algorithmic` (or `algorithm2e`)
- **Code listings:** `listings`, `xcolor`
- **Bibliography:** `biblatex` with `biber` backend
- **Hyperlinks:** `hyperref`, `url`
- **Others:** `caption`, `enumitem`, `appendix`

### Compilation Steps

#### Method 1: Using pdflatex + biber (Recommended)

```bash
cd thesis/

# First compilation pass
pdflatex main.tex

# Generate bibliography
biber main

# Two more passes to resolve references
pdflatex main.tex
pdflatex main.tex
```

#### Method 2: Using latexmk (Automated)

```bash
cd thesis/

# Compile with automatic dependency resolution
latexmk -pdf -pdflatex="pdflatex -interaction=nonstopmode" main.tex

# Clean auxiliary files
latexmk -c
```

#### Method 3: Using Overleaf

1. Create a new project on [Overleaf](https://www.overleaf.com/)
2. Upload all files maintaining the directory structure
3. Set the main document to `main.tex`
4. Click "Recompile"

### Quick Compilation Script

Create a shell script `compile.sh`:

```bash
#!/bin/bash
pdflatex -interaction=nonstopmode main.tex
biber main
pdflatex -interaction=nonstopmode main.tex
pdflatex -interaction=nonstopmode main.tex

# Clean auxiliary files
rm -f *.aux *.log *.out *.toc *.lof *.lot *.bbl *.blg *.bcf *.run.xml
echo "Compilation complete! Output: main.pdf"
```

Make it executable and run:
```bash
chmod +x compile.sh
./compile.sh
```

## Output

After successful compilation, you will get:
- **main.pdf** - The complete thesis document
- Various auxiliary files (can be cleaned with `latexmk -c`)

## Adding Figures

### Directory Structure

Place your figure files in the `figures/` directory:

```
figures/
├── chapter3/
│   ├── pinn_architecture.pdf
│   ├── system_overview.png
│   └── ...
├── chapter5/
│   ├── experimental_setup.jpg
│   ├── robot_platform.pdf
│   └── ...
└── results/
    ├── energy_comparison.pdf
    ├── path_trajectories.png
    └── ...
```

### Including Figures in LaTeX

Replace placeholder figures in the chapter files with your actual figures:

```latex
\begin{figure}[htbp]
    \centering
    \includegraphics[width=0.8\textwidth]{figures/chapter3/pinn_architecture.pdf}
    \caption{Architecture of the Physics-Informed Neural Network.}
    \label{fig:pinn_architecture}
\end{figure}
```

### Supported Image Formats

- **Vector graphics (preferred):** `.pdf`, `.eps`
- **Raster graphics:** `.png`, `.jpg`, `.jpeg`

### Figure Best Practices

1. **Use vector graphics** (PDF, EPS) for diagrams, plots, and schematics
2. **Use high-resolution raster images** (300 dpi minimum) for photographs
3. **Keep consistent styling** across all figures
4. **Include descriptive captions** and reference figures in text
5. **Use labels** for cross-referencing: `\label{fig:name}` and `\ref{fig:name}`

### Creating Plots with Python/Matplotlib

Example Python script to generate publication-quality figures:

```python
import matplotlib.pyplot as plt
import numpy as np

# Configure matplotlib for LaTeX
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'text.usetex': True,
    'figure.figsize': (6, 4),
    'figure.dpi': 300
})

# Your plotting code here
x = np.linspace(0, 10, 100)
y = np.sin(x)

plt.plot(x, y, label='Data')
plt.xlabel('Time (s)')
plt.ylabel('Energy (J)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Save as PDF for best quality
plt.savefig('figures/results/energy_plot.pdf', bbox_inches='tight')
```

## Customization Tips

### Changing Title and Author

Edit `main.tex`:

```latex
\title{Your Thesis Title}
\author{Your Name}
\date{\today}  % or specify a date
```

### Modifying Page Layout

Edit geometry settings in `main.tex`:

```latex
\usepackage[
    a4paper,
    left=3cm,
    right=3cm,
    top=3cm,
    bottom=3cm
]{geometry}
```

### Adjusting Line Spacing

Change in `main.tex`:

```latex
\usepackage{setspace}
\onehalfspacing  % or \doublespacing
```

### Customizing Headers and Footers

Modify `fancyhdr` settings in `main.tex`:

```latex
\pagestyle{fancy}
\fancyhf{}
\fancyhead[LE,RO]{\thepage}
\fancyhead[RE]{\leftmark}
\fancyhead[LO]{\rightmark}
```

### Bibliography Style

Change citation style in `main.tex`:

```latex
\usepackage[
    backend=biber,
    style=ieee,        % or numeric, authoryear, apa, etc.
    sorting=none,      % or nty, nyt, etc.
    maxbibnames=99
]{biblatex}
```

### Code Listing Customization

Modify listings settings in `appendix_a_code.tex`:

```latex
\lstset{
    basicstyle=\ttfamily\footnotesize,
    keywordstyle=\color{blue}\bfseries,
    commentstyle=\color{green!60!black}\itshape,
    stringstyle=\color{red},
    numberstyle=\tiny\color{gray},
    breaklines=true,
    showstringspaces=false,
    tabsize=4
}
```

## Common Issues and Solutions

### Issue: Bibliography not appearing

**Solution:** Make sure to run `biber` (not `bibtex`) after the first `pdflatex` run:
```bash
pdflatex main.tex
biber main        # NOT bibtex!
pdflatex main.tex
```

### Issue: References showing as "??"

**Solution:** Run `pdflatex` multiple times (at least 2-3 times) to resolve all cross-references.

### Issue: Missing packages

**Solution:** Install the full LaTeX distribution or individual packages:
```bash
# Ubuntu/Debian
sudo apt-get install texlive-full

# Or specific packages
sudo apt-get install texlive-science texlive-bibtex-extra
```

### Issue: Compilation takes too long

**Solution:** 
1. Comment out `\listoffigures` and `\listoftables` during drafts
2. Use `\includeonly{chapters/chapter1}` to compile specific chapters
3. Use draft mode: `\documentclass[draft]{report}`

### Issue: Figure not found

**Solution:** 
1. Check file path is correct relative to `main.tex`
2. Ensure figure file exists with correct extension
3. Don't include file extension in `\includegraphics` (LaTeX auto-detects)

### Issue: Overfull/underfull hbox warnings

**Solution:**
1. These are usually not critical
2. Use `\usepackage{microtype}` for better typesetting
3. Manually adjust paragraphs if needed with `\linebreak` or rewording

## Version Control

It's recommended to use Git for version control:

```bash
# Initialize repository
git init

# Create .gitignore
cat > .gitignore << EOF
*.aux
*.log
*.out
*.toc
*.lof
*.lot
*.bbl
*.blg
*.bcf
*.run.xml
*.synctex.gz
*.fdb_latexmk
*.fls
main.pdf
EOF

# Add files and commit
git add .
git commit -m "Initial thesis structure"
```

## Collaboration

### Using Overleaf with Git

Overleaf provides Git integration for easy collaboration:

1. Create project on Overleaf
2. Get Git URL from Overleaf menu
3. Clone locally: `git clone <overleaf-git-url>`
4. Work locally and push: `git push`

### Using GitHub

1. Create a private GitHub repository
2. Push your thesis: 
```bash
git remote add origin <your-github-url>
git push -u origin main
```

## Printing and Submission

### PDF/A Generation (for archiving)

Some institutions require PDF/A format:

```latex
\usepackage[a-1b]{pdfx}  % Add to preamble
```

### Two-sided Printing

Add `twoside` option:

```latex
\documentclass[12pt, twoside]{report}
```

### Checking Page Count

```bash
pdfinfo main.pdf | grep Pages
```

## Resources

### LaTeX Documentation
- [Overleaf Documentation](https://www.overleaf.com/learn)
- [LaTeX Wikibook](https://en.wikibooks.org/wiki/LaTeX)
- [CTAN (Comprehensive TeX Archive)](https://ctan.org/)

### Bibliography Management
- [BibTeX Guide](http://www.bibtex.org/)
- [Google Scholar](https://scholar.google.com/) - Export BibTeX citations
- [Zotero](https://www.zotero.org/) - Reference management software

### Figure Creation
- [TikZ Examples](https://texample.net/tikz/)
- [matplotlib](https://matplotlib.org/) - Python plotting
- [Inkscape](https://inkscape.org/) - Vector graphics editor

### Templates and Examples
- [University thesis templates](https://www.overleaf.com/latex/templates/tagged/thesis)
- [IEEE Publication Resources](https://journals.ieeeauthorcenter.ieee.org/)

## License

This thesis is the intellectual property of [Your Name] and [Your Institution]. 

The LaTeX template and structure can be reused freely. Please cite this work if you use the research content.

## Contact

**Author:** [Your Name]  
**Email:** [your.email@university.edu]  
**Website:** [your-website.com]  
**GitHub:** [github.com/yourusername]

## Acknowledgments

This thesis was typeset using LaTeX. Special thanks to the developers of:
- The LaTeX Project
- BibLaTeX and Biber
- TikZ and PGFPlots
- All open-source contributors

---

**Last Updated:** December 2024  
**LaTeX Version:** pdfTeX 3.14159265-2.6-1.40.21  
**Document Version:** 1.0
