void passAccel () {
    float accelCheck[] = {9.8345,
9.8345,
9.83928,
9.86321,
9.8345,
9.84407,
9.84885,
9.84885,
9.84407,
9.8345,
9.85843,
9.84407,
9.8345,
9.84885,
9.86321,
9.85364,
9.85364,
9.84885,
9.84885,
9.84407,
9.8345,
9.83928,
9.84407,
9.83928,
9.84407,
9.84407,
9.84407,
9.85364,
9.83928,
9.84885,
9.84885,
9.85364,
10.1073,
10.0068,
9.37507,
10.1503,
9.72443,
9.97328,
9.82492,
10.5188,
14.3569,
90.0323,
96.5647,
105.858,
100.666,
97.986,
95.708,
92.9084,
90.7693,
88.5966,
85.3137,
82.1886,
79.5469,
76.3214,
73.7898,
69.545,
64.8167,
60.2273,
55.614,
51.4313,
44.784,
22.3681,
15.3236,
12.859,
-1.33998,
-4.29272,
-9.3655,
-10.3848,
-10.5284,
-10.739,
-10.8012,
-10.337,
-11.208,
-10.6098,
-9.97807,
-9.63828,
-9.60479,
-8.94437,
-8.81994,
-8.62851,
-8.11166,
-7.91067,
-7.6666,
-7.37468,
-7.15932,
-6.84826,
-6.67597,
-6.57069,
-6.47498,
-6.14476,
-6.03948,
-5.77627,
-5.64706,
-5.50827,
-5.15892,
-5.03449,
-4.81914,
-4.666,
-4.52243,
-4.31186,
-4.12044,
-4.05822,
-3.96251,
-3.87158,
-3.6993,
-3.56052,
-3.39781,
-3.2016,
-3.21595,
-3.01017,
-2.9336,
-2.87617,
-2.77567,
-2.70867,
-2.59382,
-2.47418,
-2.48853,
-2.34975,
-2.28275,
-2.19661,
-2.03868,
-1.90947,
-1.87119,
-1.80419,
-1.71326,
-1.60319,
-1.52662,
-1.42612,
-1.35434,
-1.25862,
-1.2682,
-1.18684,
-1.14377,
-1.09113,
-1.03848,
-1.02413,
-0.995414,
-0.947557,
-0.880558,
-0.832702,
-0.799202,
-0.751346,
-0.717846,
-0.693918,
-0.626919,
-0.598205,
-0.564706,
-0.516849,
-0.44985,
-0.445065,
-0.40678,
-0.358923,
-0.339781,
-0.306281,
-0.26321,
-0.258425,
-0.229711,
-0.229711,
-0.215354,
-0.196211,
-0.191426,
-0.215354,
-0.210568,
-0.22014,
-0.224925,
-0.234496,
-0.253639,
-0.248853,
-0.287138,
-0.291924,
-0.330209,
-0.368494,
-0.382851,
-0.421136,
-0.454636,
-0.478564,
-0.507278,
-0.55992,
-0.555135,
-0.588634,
-0.588634,
-0.574277,
-0.569491,
-0.540777,
-0.507278,
-0.48335,
-0.473778,
-0.464207,
-0.473778,
-0.502492,
-0.488136,
-0.526421,
-0.526421,
-0.550349,
-0.59342,
-0.598205,
-0.626919,
-0.626919,
-0.66999,
-0.708275,
-0.751346,
-0.794416,
-0.885344,
-0.909272,
-0.976271,
-1.02891,
-1.13898,
-1.15334,
-1.12941,
-1.21555,
-1.2682,
-1.37348,
-1.40219,
-1.44526,
-1.47876,
-1.50748,
-1.57448,
-1.5984,
-1.62233,
-1.64626,
-1.6989,
-1.77069,
-1.84726,
-1.8664,
-1.94297,
-1.94297,
-2.0004,
-2.0004,
-1.30169,
-3.27816,
-5.13499,
-5.95334,
-7.73838,
-13.6678,
-12.3326,
-10.5428,
-9.22193,
-8.17866,
-9.77228,
-9.87278,
-7.48953,
-9.10229,
-7.71445,
-9.18365,
-10.916,
-11.7727,
-11.0788,
-9.61914,
-13.9358,
-9.63828,
-12.347,
-8.43709,
-8.86779,
-8.83908,
-13.0887,
-9.48993,
-11.5382,
-8.66201,
-11.677,
-9.64307,
-8.2983,
-9.92064,
-9.84407,
-10.0355,
-10.0881,
-8.78165,
-9.46121,
-9.17408,
-9.42293,
-9.93021,
-8.43709,
-8.48973,
-10.1408,
-9.60957,
-9.35593,
-10.9878,
-8.93958,
-12.2943,
-9.94935,
-8.51366,
-11.5717,
-9.82492,
-9.79621,
-11.9545,
-7.5326,
-12.7107,
-9.88714,
-8.03031,
-11.6913,
-9.23629,
-9.77228,
-9.83928,
-9.21236,
-10.4901,
-9.62871,
-8.78165,
-10.7916,
-9.97328,
-9.20279,
-9.95414,
-10.1791,
-8.60937,
-9.85843,
-10.337,
-9.72443,
-10.27,
-10.3753,
-10.3992,
-8.56151,
-8.28395,
-9.74835,
-8.26002,
-10.1503,
-9.30807,
-8.72422,
-8.4323,
-7.8676,
-7.89631,
-9.04965,
-8.15474,
-8.91087,
-9.84885,
-8.34137,
-10.3848,
-7.06361,
-5.85284,
-6.23569,
-10.3131,
-9.07837,
-8.38923,
-10.0403,
-10.9974,
-8.23609,
-8.52802,
-8.08773,
-7.86281,
-10.9017,
-9.40379,
-12.5958,
-7.3986,
-7.70967,
-6.96311,
-8.4323,
-10.9065,
-8.30787,
-10.8203,
-7.48474,
-8.86301,
-13.6487,
-7.37468,
-7.89631};
}

void passAlt () {
    float altCheck[] = {2.49149,
3.16757,
2.49149,
2.49149,
3.25208,
2.55487,
2.55487,
2.49149,
1.8154,
2.89291,
2.89291,
2.99857,
2.66052,
2.55487,
2.49149,
2.49149,
2.21683,
2.21683,
2.32248,
2.99857,
3.16757,
2.89291,
2.82953,
2.89291,
2.32248,
2.66052,
2.82953,
2.32248,
2.82953,
2.49149,
3.16757,
3.33661,
2.49149,
3.33661,
2.55487,
1.8154,
3.16757,
2.49149,
2.82953,
1.8154,
2.89291,
3.67465,
5.36484,
5.25922,
8.47058,
10.689,
14.7243,
18.7175,
22.5204,
26.9149,
34.1194,
37.9224,
45.2959,
53.7892,
61.6064,
68.4728,
78.4978,
86.0826,
95.2731,
105.837,
113.348,
123.595,
133.334,
141.722,
152.339,
162.015,
171.448,
180.121,
188.023,
201.09,
205.052,
214.95,
224.87,
230.046,
242.131,
247.74,
255.441,
262.962,
271.445,
278.629,
286.182,
293.735,
300.231,
308.323,
313.626,
319.531,
326.768,
333.465,
341.219,
347.494,
353.526,
359.452,
365.399,
370.47,
374.833,
383.146,
387.266,
392.95,
398.57,
403.144,
408.922,
413.655,
419.623,
424.895,
428.761,
433.599,
437.909,
442.483,
445.219,
450.586,
455.868,
459.671,
463.273,
467.857,
472.093,
476.689,
480.745,
483.756,
486.576,
490.992,
494.362,
497.711,
500.732,
504.63,
507.82,
510.651,
513.493,
517.032,
519.155,
521.458,
524.215,
526.962,
530.511,
531.229,
534.589,
536.913,
539.564,
542.511,
543.114,
546.145,
547.572,
550.498,
551.839,
553.783,
554.85,
558.051,
558.325,
560.375,
561.97,
562.678,
563.227,
564.653,
566.238,
567.664,
569.09,
569.734,
570.791,
571.308,
572.228,
573.199,
574.076,
573.125,
574.076,
574.985,
574.636,
575.428,
575.175,
577.288,
576.495,
576.22,
576.928,
576.052,
575.692,
575.502,
575.344,
575.428,
574.076,
573.728,
573.368,
572.661,
572.301,
572.661,
570.875,
569.999,
568.741,
568.213,
568.108,
565.53,
564.389,
563.586,
563.037,
561.252,
559.836,
558.325,
556.276,
554.85,
551.998,
551.216,
548.797,
546.336,
545.596,
542.131,
540.631,
539.205,
536.797,
534.673,
527.384,
525.525,
523.042,
520.74,
517.549,
515.701,
512.944,
509.405,
507.271,
503.193,
500.542,
496.844,
493.042,
491.077,
488.52,
484.622,
481.633,
478.432,
474.819,
471.037,
467.604,
463.632,
445.821,
441.585,
440.719,
435.258,
431.74,
420.49,
408.552,
404.528,
396.626,
392.591,
388.999,
385.935,
381.382,
377.368,
373.164,
370.1,
366.171,
363.107,
359.536,
357.603,
354.117,
347.832,
342.687,
338.758,
337.363,
335.197,
330.75,
326.746,
322.299,
316.309,
312.992,
307.964,
303.094,
300.316,
296.661,
293.449,
290.946,
287.639,
283.91,
281.396,
278.111,
275.153,
271.097,
267.188,
263.554,
259.751,
256.371,
250.413,
248.416,
242.279,
238.402,
233.997,
232.19,
228.472,
225.63,
221.225,
217.01,
213.556,
209.172,
206.753,
202.802,
198.671,
195.999,
192.481,
190.421,
188.086,
184.738,
178.04,
174.776,
171.501,
168.86,
166.019,
161.223,
155.899,
152.053,
150.258,
145.969,
142.884,
140.148,
136.039,
132.458,
130.408,
126.119,
123.996,
119.813,
116.401,
110.601,
106.925,
103.344,
99.4247,
97.4598,
94.9139,
90.0546,
87.4137,
81.2867,
77.8006,
74.9167,
72.6138,
67.9446,
64.036,
60.4021,
58.6908,
53.599,
51.7398,
49.4369,
44.6198,
41.5563,
36.6547,
32.4081,
29.8728,
24.105,
19.8795,
15.0624,
11.682,
7.70999,
5.93527,
2.30136,
-1.39597,
-8.41031,
-9.33994,
-12.2978,
-16.9247,
-20.8967};
}