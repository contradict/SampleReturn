all: SystemDiagram.pdf NavigationBlockDiagram.pdf

NavigationBlockDiagram.pdf: NavigationBlockDiagram.dot
	dot -Tpdf -o $@ $<

%.pdf: %.dot
	circo -Tpdf -o $@ $<
