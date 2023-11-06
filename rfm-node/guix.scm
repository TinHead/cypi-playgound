(use-modules 
      (guix packages)
      (guix download)
      (guix licenses)
      (gnu packages python)
      (gnu packages python-xyz)
     (guix build-system copy)
     (guix build-system pyproject)
     (gnu packages python-build)
     (gnu packages check)
     (guix build-system python)
     (th packages helix-editor))

(define hatch-reqs
(package
  (name "python-hatch-requirements-txt")
  (version "0.4.0")
  (source
   (origin
     (method url-fetch)
     (uri (pypi-uri "hatch_requirements_txt" version))
     (sha256
      (base32 "0pa5k2nrpzfi5m7a0w78f81vavf37qianbr4fdnybnc5dsa0j1c0"))))
  (build-system pyproject-build-system)
  (home-page "")
  (inputs 
     `(
      ; ("python-pytest" ,python-pytest)
      ("python-hatchling" ,python-hatchling)))
  (arguments
    ;; Broken tests or cyclic dependecies with other packages.
    '(#:phases
      (modify-phases %standard-phases
                     (delete 'sanity-check))
      #:tests? #f)) 
  (synopsis
   "Hatchling plugin to read project dependencies from requirements.txt")
  (description
   "Hatchling plugin to read project dependencies from requirements.txt")
  (license #f))
)

(define python-mpremote
(package
  (name "python-mpremote")
  (version "1.21.0")
  (source
   (origin
     (method url-fetch)
     (uri (pypi-uri "mpremote" version))
     (sha256
      (base32 "0qgdajfah50df1sqq7gp5izgyi07bwm4cndb07lrkx3g3x8r9g35"))))
  (build-system pyproject-build-system)
  (home-page "")
  (inputs 
     `(
      ("python-hatch-vcs" ,python-hatch-vcs)
      ("hatch-reqs" ,hatch-reqs)
      ("python-pip" ,python-pip)
      ("python-pyserial" ,python-pyserial)
      ("python-hatchling" ,python-hatchling))) 
    (arguments
    ;; Broken tests or cyclic dependecies with other packages.
    '(#:phases
      (modify-phases %standard-phases
                     (delete 'sanity-check))
      #:tests? #f))  
  (synopsis "Tool for interacting remotely with MicroPython devices")
  (description "Tool for interacting remotely with @code{MicroPython} devices")
  (license expat)))
  

(define circuitpython-stubs
(package
  (name "python-circuitpython-stubs")
  (version "8.2.7")
  (source
   (origin
     (method url-fetch)
     (uri (pypi-uri "circuitpython-stubs" version))
     (sha256
      (base32 "0d41ji5vhd3rsxa9x6cngxmlicifg8dn24q36gmhhrn6a2q6kyjm"))))
  (build-system python-build-system)
     (arguments
        ; '(#:install-plan '(("./" "lib/python3.10/site-packages/micropython-rp2-pico-stubs" ))
        '(#:phases
          (modify-phases %standard-phases
            (delete 'build)
            (replace 'install ;; Replace the install step with the function defined below
               (lambda* (#:key outputs #:allow-other-keys)
                 (let* ((outlib (string-append (assoc-ref outputs "out") "/lib/python3.10/site-packages")))
               (copy-recursively "." outlib))))
            (delete 'check)
            (delete 'sanity-check))
          #:tests? #f))
     (native-search-paths
       (list (search-path-specification
            (variable "GUIX_PYTHONPATH")
            (files (list "/lib/python3.10/site-packages")))))
  (inputs 
    `(("python" ,python))
      ; ("helix-editor-bin" ,helix-editor-bin)
      ; ("python-lsp-server", python-lsp-server)
      ; ("python-mpremote" ,python-mpremote))
      )
  (home-page "https://github.com/adafruit/circuitpython")
  (synopsis "PEP 561 type stubs for CircuitPython")
  (description "PEP 561 type stubs for @code{CircuitPython}")
  (license expat))
)

; (define-public micropython-rp2-pico-w-stubs
;   (package
;     (name "python-micropython-rp2-stubs")
;     (version "1.20.0.post3")
;     (source
;      (origin
;        (method url-fetch)
;        (uri (pypi-uri "micropython_rp2_stubs" version))
;        (sha256
;         (base32 "095ih3832isy10dz54wjrfgpqay3k8l9rb6sqm4sx12l2s2c19bi"))))
;     (build-system python-build-system)
;     (inputs
;       `(("python" ,python)
;         ("helix-editor-bin" ,helix-editor-bin)
;         ("python-lsp-server", python-lsp-server)
;         ("python-mpremote" ,python-mpremote)))
;      (arguments
;         ; '(#:install-plan '(("./" "lib/python3.10/site-packages/micropython-rp2-pico-stubs" ))
;         '(#:phases
;           (modify-phases %standard-phases
;             (delete 'build)
;             (replace 'install ;; Replace the install step with the function defined below
;                (lambda* (#:key outputs #:allow-other-keys)
;                  (let* ((outlib (string-append (assoc-ref outputs "out") "/lib/python3.10/site-packages")))
;                (copy-recursively "." outlib))))
;             (delete 'check)
;             (delete 'sanity-check))
;           #:tests? #f))
;      (native-search-paths
;        (list (search-path-specification
;             (variable "GUIX_PYTHONPATH")
;             (files (list "/lib/python3.10/site-packages")))))
;     (home-page "https://github.com/josverl/micropython-stubs#micropython-stubs")
;     (synopsis "MicroPython stubs")
;     (description "@code{MicroPython} stubs")
;     (license expat))
; )

(define mock-package
  (package
    (name "mock-package")
    (version "0.0.1")
    (source #f)
    (build-system copy-build-system)
    (inputs
      `(("python" ,python)
        ("helix-editor-bin" ,helix-editor-bin)
        ("python-lsp-server", python-lsp-server)
        ("python-mpremote" ,python-mpremote)
        ("circuitpython-stubs" ,circuitpython-stubs)))
    (synopsis "")
    (description "")
    (home-page "")
    (license #f)))


mock-package

