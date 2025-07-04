# Add set(CONFIG_USE_driver_powerquad true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

      target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
          ${CMAKE_CURRENT_LIST_DIR}/fsl_powerquad_data.c
          ${CMAKE_CURRENT_LIST_DIR}/fsl_powerquad_basic.c
          ${CMAKE_CURRENT_LIST_DIR}/fsl_powerquad_math.c
          ${CMAKE_CURRENT_LIST_DIR}/fsl_powerquad_matrix.c
          ${CMAKE_CURRENT_LIST_DIR}/fsl_powerquad_filter.c
          ${CMAKE_CURRENT_LIST_DIR}/fsl_powerquad_transform.c
        )

  
      target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
          ${CMAKE_CURRENT_LIST_DIR}/.
        )

  
