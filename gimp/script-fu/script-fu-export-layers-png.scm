(define (export-layer-png image layer outfile)
  ;;(gimp-message
  ;;  (string-append
  ;;    "export-layer-png(image: " (number->string image)
  ;;    ", layer: " (number->string layer)
  ;;    ", outfile: " outfile ")"))
  (file-png-save-defaults RUN-NONINTERACTIVE image layer outfile outfile)
)

(define (export-layers-png image layers outdir)
  (if (not (null? layers))
    (let*
      (
        (firstLayer (car layers))
        (fname (string-append (car (gimp-layer-get-name firstLayer)) ".png"))
        (fpath (string-append outdir "/" fname))
      )
      ; output the first layer in the list
      (export-layer-png image firstLayer fpath)
      ; recurse of the rest of the list
      (export-layers-png image (cdr layers) outdir)
    )
    ()
  )
)

(define (array->list size array)
  (let*
    (
      (current-idx (- size 1))
      (newlist '())
    )
    (while (> current-idx -1)
      (set! newlist (cons (aref array current-idx) newlist))
      (set! current-idx (- current-idx 1))
    )
    newlist ; return the new list
  )
)

(define (script-fu-export-layers-png image outdir)
  (let*
    (
      (layers-info (gimp-image-get-layers image))
      (layers (array->list (car layers-info) (cadr layers-info)))
    )
  ;;(gimp-message (string-append "image:  " (number->string image)))
  ;;(gimp-message (string-append "outdir: " outdir))
  ;;(gimp-message
  ;;  (string-append
  ;;    "layers: ("
  ;;    (apply string-append
  ;;      (map (lambda (x) (string-append (number->string x) ", ")) layers))
  ;;    ")"))
  (export-layers-png image layers outdir)
))

(script-fu-register
  "script-fu-export-layers-png"              ; func name
  "Export layers to file as PNG"             ; menu label
  "Save each layer to a separate file\
    using the layer's name as a png file."   ; description
  "Michael Bentley"                          ; author
  "copyright 2020, Michael Bentley"          ; copyright notice
  "April 13, 2020"                           ; date created
  "RGBA"                                     ; image type applicability
  SF-IMAGE    "Current Image" 0              ; current image handle
  ;SF-DRAWABLE "Current Layer" 17             ; current image layer handle
  SF-DIRNAME  "Output Directory"  "/tmp"     ; directory parameter
)
(script-fu-menu-register "script-fu-export-layers-png" "<Image>/File/Export Layers")
