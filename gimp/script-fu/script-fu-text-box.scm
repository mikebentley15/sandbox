(define (script-fu-text-box text font fontSize color)
  (let*
    (
      (width 10)
      (height 10)
      (image (car (gimp-image-new width height RGB)))
      (layer (car (gimp-layer-new
                    image width height RGB-IMAGE
                    "layer 1" 100 LAYER-MODE-NORMAL))))
    (gimp-image-add-layer image layer 0)
    (gimp-context-set-background '(255 255 255))
    (gimp-context-set-foreground color)
    (gimp-drawable-fill layer BACKGROUND-FILL)
    (set! text
      (car (gimp-text-fontname
             image layer 0 0 text 0 TRUE fontSize PIXELS "Sans")))
    (set! width  (car (gimp-drawable-width  text)))
    (set! height (car (gimp-drawable-height text)))
    (gimp-image-resize image width height 0 0)
    (gimp-layer-resize layer width height 0 0)
    (gimp-display-new image)
    ;(gimp-image-clean-all image)
    ))

(script-fu-register
  "script-fu-text-box"                       ; func name
  "Text Box"                                 ; menu label
  "Creates a simple text box, sized to fit\
    around the user's choice of text,\
    font, font size, and color."             ; description
  "Michael Bentley"                          ; author
  "copyright 2020, Michael Bentley"          ; copyright notice
  "April 13, 2020"                           ; date created
  ""                                         ; image type applicability
  SF-STRING      "Text"       "Text Box"     ; a string variable
  SF-FONT        "Font"       "Charter"      ; a font variable
  SF-ADJUSTMENT  "Font size"  '(50 1 1000 1 10 0 1)
                                             ; a spin button
  SF-COLOR       "Color"      '(0 0 0)       ; a color variable
)
(script-fu-menu-register "script-fu-text-box" "<Image>/File/Create/Text")
