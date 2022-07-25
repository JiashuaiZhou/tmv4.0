
all: mpeg-vmesh-sw-manual.pdf

## mpegdoc.cls : https://mpeg.expert/software/dflynn/mpeg-report

MD_SRC=\
	README.about.md \
	README.architecture.md \
	README.build.md \
	README.clone.md \
	README.dec_params.md \
	README.enc_params.md \
	README.usage.md \
	$(NULL)

TEX_SRC=\
	mpeg-vmesh-sw-manual.tex \
	$(patsubst %.md,%.tex,$(MD_SRC)) \
	$(NULL)

mpeg-vmesh-sw-manual.pdf: $(TEX_SRC)

LATEX:=$(shell which xelatex || which pdflatex || which latex)

# for bibtex
%.aux: %.tex
	$(LATEX) $(LATEXFLAGS) $<

# normally this has a rule to use bibtool to pull in mpeg bib entries
%.bib: %.tex
	touch $@

%.bbl: %.tex %.aux %.bib
	-bibtex $(*F)

ifneq ($(notdir $(LATEX)),latex)
# for direct pdf generation
%.pdf: %.tex %.bbl
	$(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<
endif

%.pdf: %.dvi
	dvipdfm $*.dvi

%.dvi: %.tex %.bbl
	$(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<

# convert markdown text to latex for document generation
%.tex: %.md
	pandoc -f markdown -t latex -o $@ $<

clean:
	rm -f $(patsubst %.md,%.tex,$(MD_SRC)) \
		 mpeg-vmesh-sw-manual.pdf \
		 mpeg-vmesh-sw-manual.log \
		 mpeg-vmesh-sw-manual.out \
		 mpeg-vmesh-sw-manual.toc \
		 mpeg-vmesh-sw-manual.blg