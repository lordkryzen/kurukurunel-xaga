/* SPDX-License-Identifier: GPL-2.0 */
/* Original xaga MT6375 connector NTC: 24.9 kohm pull-up, 1.8 V reference.
 * Keep the original resistance/temperature table and integer interpolation. */
#ifndef MT6375_LEGACY_TS_NTC_H
#define MT6375_LEGACY_TS_NTC_H
struct ntc_desc {
	int temp;	/* 0.1 deg.C */
	int res;	/* ohm */
};

static const struct ntc_desc ts_ntc_table[] = {
	{-400, 4251000},
	{-350, 3005000},
	{-300, 2149000},
	{-250, 1554000},
	{-200, 1135000},
	{-150, 837800},
	{-100, 624100},
	{-50,  469100},
	{0,    355600},
	{50,   271800},
	{100,  209400},
	{150,  162500},
	{200,  127000},
	{250,  100000},
	{300,  79230},
	{350,  63180},
	{400,  50680},
	{450,  40900},
	{500,  33190},
	{550,  27090},
	{600,  22220},
	{650,  18320},
	{700,  15180},
	{750,  12640},
	{800,  10580},
	{850,  8887},
	{900,  7500},
	{950,  6357},
	{1000, 5410},
	{1050, 4623},
	{1100, 3966},
	{1150, 3415},
	{1200, 2952},
	{1250, 2561}
};

static int mt6375_ts_uv_to_temp(int uv, int *temperature)
{
	int millivolts, resistance, lower, upper;
	int count = ARRAY_SIZE(ts_ntc_table);

	/* A zero/rail ADC is unavailable/short/open, not a valid hot sample. */
	if (uv <= 0 || uv >= 1800000)
		return -ERANGE;
	millivolts = uv / 1000;
	resistance = 24900 * millivolts / (1800 - millivolts);
	if (resistance >= ts_ntc_table[0].res) {
		*temperature = ts_ntc_table[0].temp;
		return 0;
	}
	if (resistance <= ts_ntc_table[count - 1].res) {
		*temperature = ts_ntc_table[count - 1].temp;
		return 0;
	}
	for (upper = 1; upper < count; upper++)
		if (resistance >= ts_ntc_table[upper].res)
			break;
	lower = upper - 1;
	*temperature = (ts_ntc_table[lower].temp * (resistance - ts_ntc_table[upper].res) +
		ts_ntc_table[upper].temp * (ts_ntc_table[lower].res - resistance)) /
		(ts_ntc_table[lower].res - ts_ntc_table[upper].res);
	return 0;
}
#endif
