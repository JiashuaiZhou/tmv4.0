
all: mpeg-vmesh-sw-manual.pdf README.md cleantmpfiles

## mpegdoc.cls : https://mpeg.expert/software/dflynn/mpeg-report

MD_SRC=\
	readme/about.md \
	readme/architecture.md \
	readme/build.md \
	readme/clone.md \
	readme/dec_params.md \
	readme/enc_params.md \
	readme/met_params.md \
	readme/usage.md \
	readme/contact.md \
	$(NULL)

TEX_SRC=\
	mpeg-vmesh-sw-manual.tex \
	$(patsubst %.md,%.tex,$(MD_SRC)) \
	$(NULL)

mpeg-vmesh-sw-manual.pdf: $(TEX_SRC)

LATEX:=$(shell which xelatex || which pdflatex || which latex) 

readme/enc_params.md:
	bash ./update_input_parameters.sh ../build/Release/bin/encode > readme/enc_params.md
	@echo create readme/enc_params.md

readme/dec_params.md:
	bash ./update_input_parameters.sh ../build/Release/bin/decode > readme/dec_params.md
	@echo create readme/dec_params.md

readme/met_params.md:
	bash ./update_input_parameters.sh ../build/Release/bin/metrics > readme/met_params.md
	@echo create readme/met_params.md

# for bibtex
%.aux: %.tex
	@echo $(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<

# normally this has a rule to use bibtool to pull in mpeg bib entries
%.bib: %.tex
	touch $@

%.bbl: %.tex %.aux %.bib
	-bibtex $(*F)

ifneq ($(notdir $(LATEX)),latex)
# for direct pdf generation
%.pdf: %.tex %.bbl
	@echo $(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<
endif

%.pdf: %.dvi
	@echo dvipdfm $*.dvi
	dvipdfm $*.dvi

%.dvi: %.tex %.bbl
	@echo $(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<

# convert markdown text to latex for document generation
%.tex: %.md
	@echo pandoc -f markdown -t latex -o $@ $<
	pandoc -f markdown -t latex -o $@ $<

README.md: $(patsubst %.md,%.tex,$(MD_SRC))
	@echo ./build-doc.sh
	./build-doc.sh

cleantmpfiles:
	@echo cleantmpfiles
	rm -f $(patsubst %.md,%.tex,$(MD_SRC)) \
		 mpeg-vmesh-sw-manual.log \
		 mpeg-vmesh-sw-manual.out \
		 mpeg-vmesh-sw-manual.toc \
		 mpeg-vmesh-sw-manual.blg \
		 mpeg-vmesh-sw-manual.bbl \
		 mpeg-vmesh-sw-manual.bib \
		 mpeg-vmesh-sw-manual.aux \
		 readme/enc_params.md \
		 readme/dec_params.md \
		 readme/met_params.md

clean: cleantmpfiles
	@echo clean
	rm -f mpeg-vmesh-sw-manual.pdf