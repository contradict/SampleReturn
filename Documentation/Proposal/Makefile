
proposal.pdf: proposal.tex proposal.bbl data_flow.dot.pdf
	pdflatex proposal.tex

proposal.bbl: proposal.aux main.bib
	bibtex proposal.aux

proposal.aux: data_flow.dot.pdf
	pdflatex proposal.tex

data_flow.dot.pdf: data_flow.dot
	dot -Tpdf -O $<

clean:
	rm *.pdf *.aux *.log *.bbl *.blg

.PHONY: clean
