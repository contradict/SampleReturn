
proposal.pdf: proposal.tex proposal.bbl data_flow.dot.pdf tables
	pdflatex proposal.tex

proposal.bbl: proposal.aux main.bib
	bibtex proposal.aux

proposal.aux: data_flow.dot.pdf
	pdflatex proposal.tex

data_flow.dot.pdf: data_flow.dot
	dot -Tpdf -O $<

tables: powerbudget_template.tex massbudget_template.tex mktables
	scripts/GetTableData.py credentials
	touch tables

clean:
	rm build/* *.pdf

.PHONY: clean
