#!/bin/sh
platex document.tex
pbibtex document
dvipdfmx document
