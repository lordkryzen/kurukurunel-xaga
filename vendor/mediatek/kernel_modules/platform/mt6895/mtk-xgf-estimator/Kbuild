obj-m += mtk_xgf_estimator.o
mtk_xgf_estimator-y := estimator.o native_algorithm.o
quiet_cmd_xgf_blob = XGF_BLOB $@
      cmd_xgf_blob = python3 $(src)/prepare_native_algorithm.py --cc "$(CC)"
targets += native_algorithm.o
$(obj)/native_algorithm.o: $(src)/prebuilt/xgf_v3_ko_quark.o $(src)/prepare_native_algorithm.py FORCE
	$(call if_changed,xgf_blob)

# Device module selection.
obj-m := mtk_xgf_estimator.o
