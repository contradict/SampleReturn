all: SystemDiagram.pdf

%.pdf: %.dot
	circo -Tpdf -o $@ $<
