;;; package ---- Summary

;;;; Commentary:
; Optional Emacs configuration for marker_alignemnt
; Still uses user default init file, but loads extra config

;(load "~/.emacs")

; Add out of source build path to the M-x compile's path
(add-to-list 'compilation-search-path "~/sandbox/marker_alignment_debug")
