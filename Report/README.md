# Template for HRL Reports

This is a template to create a report for a course offered by the Humanoid Robots Lab (HRL).
The template is designed for students of the following courses:
- Project Group Humanoide Roboter (BA-INF 051)
- Lab Course Humanoid Robots (MA-INF 4214)
- Seminar Humanoid Robots (MA-INF 4213)

# Setup and build
### With git: Execute before first commit
Run the script `setup_autoformat.sh` and select option 1 before your first commit!
```
chmod +x setup-autoformat.sh
./setup-autoformat.sh
```
Select 1 and hit the enter key to format one sentence per line.

### Changes
Choose the correct course and term (winter term/summer term) with the corresponding year.
Insert your name, title, matriculation number, and supervisor in the `hrl_report.tex` file in the code section `Course and student information`.

### Build the pdf

To build the pdf from the tex file you can execute the Makefile with
```
make
```

# Notes on academic writing

* Use a spell checker with US English as spelling language
* Use a [Academic Writing Checker](https://github.com/devd/Academic-Writing-Check)

# Notes regarding figures
All images go to the subfolder figures.

# Notes regarding bib entries
It is recommended to use a tool to organize your related work. You can use [Zotero](https://www.zotero.org/) to autogenerate your `bibliography.bib` file. Furthermore, it is recommended to use the Zotero plugin [Better BibTex](https://retorque.re/zotero-better-bibtex/installation/) to optimize the generation of bib keys. When doing so, not all fields are required. In the export settings of `Better BibTex` you insert the following keys no omit unrequired fields: `eprint,eprinttype,primaryclass,archiveprefix,address,copyright,month,editor,location,publisher,pages,volume,number,isbn,issn,doi,abstract,langid,file,keywords,note`

# LaTeX and TeXstudio installation on Linux (Ubuntu)

To install LaTeX on Ubuntu execute
```
sudo apt update
sudo apt install texlive-full
```

To install the LaTeX editor TeXstudio execute
```
sudo apt install texstudio
```