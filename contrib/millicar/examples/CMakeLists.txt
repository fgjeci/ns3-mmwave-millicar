set(base_examples
    vehicular-5g-24
    vehicular-5g-48
    vehicular-5g-64
    vehicular-5g-16
    vehicular-5g-8
    vehicular-5g-few-users
    # vehicular-5g
    # vehicular-5g-single-group
    # vehicular-5g-multi-groups
    # vehicular-5g-multi-groups_mobility
    
    # vehicular-simple-one
    # vehicular-simple-two
    # vehicular-simple-three
    # vehicular-simple-four
)

foreach(
  example
  ${base_examples}
)
  build_lib_example(
    NAME ${example}
    SOURCE_FILES ${example}.cc
    LIBRARIES_TO_LINK 
        ${libmillicar}
        ${libcore}
        ${libinternet}
        ${libpropagation}
        ${libspectrum}        
        ${libmmwave}
        ${libbuildings}
  )
endforeach()
