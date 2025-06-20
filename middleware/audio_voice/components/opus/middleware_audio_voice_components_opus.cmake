# Add set(CONFIG_USE_middleware_audio_voice_components_opus true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

      target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
          ${CMAKE_CURRENT_LIST_DIR}/celt/bands.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/celt.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/celt_decoder.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/celt_encoder.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/celt_lpc.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/cwrs.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/entcode.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/entdec.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/entenc.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/kiss_fft.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/laplace.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/mathops.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/mdct.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/modes.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/pitch.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/quant_bands.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/rate.c
          ${CMAKE_CURRENT_LIST_DIR}/celt/vq.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/A2NLSF.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/CNG.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/HP_variable_cutoff.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/LPC_analysis_filter.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/LPC_fit.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/LPC_inv_pred_gain.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/LP_variable_cutoff.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NLSF2A.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NLSF_VQ.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NLSF_VQ_weights_laroia.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NLSF_decode.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NLSF_del_dec_quant.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NLSF_encode.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NLSF_stabilize.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NLSF_unpack.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NSQ.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/NSQ_del_dec.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/PLC.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/VAD.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/VQ_WMat_EC.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/ana_filt_bank_1.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/biquad_alt.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/bwexpander.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/bwexpander_32.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/check_control_input.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/code_signs.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/control_SNR.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/control_audio_bandwidth.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/control_codec.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/debug.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/dec_API.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/decode_core.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/decode_frame.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/decode_indices.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/decode_parameters.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/decode_pitch.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/decode_pulses.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/decoder_set_fs.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/enc_API.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/encode_indices.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/encode_pulses.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/gain_quant.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/init_decoder.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/init_encoder.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/inner_prod_aligned.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/interpolate.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/lin2log.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/log2lin.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/pitch_est_tables.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/process_NLSFs.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/quant_LTP_gains.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/resampler.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/resampler_down2.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/resampler_down2_3.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/resampler_private_AR2.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/resampler_private_IIR_FIR.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/resampler_private_down_FIR.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/resampler_private_up2_HQ.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/resampler_rom.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/shell_coder.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/sigm_Q15.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/sort.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/stereo_LR_to_MS.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/stereo_MS_to_LR.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/stereo_decode_pred.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/stereo_encode_pred.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/stereo_find_predictor.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/stereo_quant_pred.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/sum_sqr_shift.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/table_LSF_cos.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/tables_LTP.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/tables_NLSF_CB_NB_MB.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/tables_NLSF_CB_WB.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/tables_gain.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/tables_other.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/tables_pitch_lag.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/tables_pulses_per_block.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/LTP_analysis_filter_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/LTP_scale_ctrl_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/apply_sine_window_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/autocorr_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/burg_modified_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/corrMatrix_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/encode_frame_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/find_LPC_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/find_LTP_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/find_pitch_lags_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/find_pred_coefs_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/k2a_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/k2a_Q16_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/noise_shape_analysis_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/pitch_analysis_core_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/process_gains_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/regularize_correlations_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/residual_energy16_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/residual_energy_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/schur64_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/schur_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/vector_ops_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed/warped_autocorrelation_FIX.c
          ${CMAKE_CURRENT_LIST_DIR}/src/opus.c
          ${CMAKE_CURRENT_LIST_DIR}/src/opus_cci.c
          ${CMAKE_CURRENT_LIST_DIR}/src/opus_cci_raw.c
          ${CMAKE_CURRENT_LIST_DIR}/src/opus_decoder.c
          ${CMAKE_CURRENT_LIST_DIR}/src/opus_encoder.c
          ${CMAKE_CURRENT_LIST_DIR}/src/opus_multistream.c
          ${CMAKE_CURRENT_LIST_DIR}/src/opus_multistream_decoder.c
          ${CMAKE_CURRENT_LIST_DIR}/src/opus_multistream_encoder.c
          ${CMAKE_CURRENT_LIST_DIR}/src/repacketizer.c
        )

  
      target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
          ${CMAKE_CURRENT_LIST_DIR}/celt
          ${CMAKE_CURRENT_LIST_DIR}/include
          ${CMAKE_CURRENT_LIST_DIR}/silk
          ${CMAKE_CURRENT_LIST_DIR}/silk/fixed
          ${CMAKE_CURRENT_LIST_DIR}/src
        )

    if(CONFIG_USE_COMPONENT_CONFIGURATION)
  message("===>Import configuration from ${CMAKE_CURRENT_LIST_FILE}")

      target_compile_definitions(${MCUX_SDK_PROJECT_NAME} PUBLIC
                  -DHAVE_CONFIG_H
              )
  
  
  endif()

