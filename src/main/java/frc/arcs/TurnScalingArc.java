package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class TurnScalingArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (2.00,13.50,0.00)
	// (5.00,16.50,89.99)
	
    public TurnScalingArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public TurnScalingArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.000,0.000,10.000,0.000},
				{0.271,2.714,10.000,0.000},
				{0.814,5.427,10.000,0.000},
				{1.628,8.141,10.000,0.000},
				{2.714,10.854,10.000,0.000},
				{4.070,13.568,10.000,0.000},
				{5.698,16.281,10.000,0.000},
				{7.598,18.995,10.000,0.000},
				{9.769,21.708,10.000,0.000},
				{12.211,24.422,10.000,0.000},
				{14.924,27.135,10.000,0.000},
				{17.909,29.849,10.000,0.010},
				{21.166,32.562,10.000,0.010},
				{24.693,35.276,10.000,0.010},
				{28.492,37.989,10.000,0.010},
				{32.562,40.703,10.000,0.020},
				{36.904,43.416,10.000,0.020},
				{41.517,46.130,10.000,0.030},
				{46.401,48.844,10.000,0.030},
				{51.557,51.557,10.000,0.040},
				{56.984,54.271,10.000,0.050},
				{62.682,56.984,10.000,0.060},
				{68.652,59.698,10.000,0.080},
				{74.893,62.411,10.000,0.090},
				{81.406,65.125,10.000,0.110},
				{88.190,67.838,10.000,0.120},
				{95.245,70.552,10.000,0.140},
				{102.571,73.265,10.000,0.170},
				{110.169,75.979,10.000,0.190},
				{118.038,78.692,10.000,0.220},
				{126.179,81.406,10.000,0.250},
				{134.591,84.119,10.000,0.290},
				{143.274,86.833,10.000,0.330},
				{152.229,89.546,10.000,0.370},
				{161.455,92.260,10.000,0.410},
				{170.952,94.973,10.000,0.460},
				{180.721,97.687,10.000,0.520},
				{190.761,100.401,10.000,0.580},
				{201.072,103.114,10.000,0.640},
				{211.655,105.828,10.000,0.710},
				{222.509,108.541,10.000,0.780},
				{233.635,111.255,10.000,0.860},
				{245.032,113.968,10.000,0.950},
				{256.700,116.682,10.000,1.040},
				{268.639,119.395,10.000,1.140},
				{280.850,122.109,10.000,1.250},
				{293.332,124.822,10.000,1.360},
				{306.086,127.536,10.000,1.490},
				{319.111,130.249,10.000,1.620},
				{332.407,132.963,10.000,1.750},
				{345.975,135.676,10.000,1.900},
				{359.814,138.390,10.000,2.060},
				{373.924,141.103,10.000,2.230},
				{388.306,143.817,10.000,2.410},
				{402.959,146.531,10.000,2.600},
				{417.883,149.244,10.000,2.800},
				{433.079,151.958,10.000,3.010},
				{448.546,154.671,10.000,3.240},
				{464.285,157.385,10.000,3.480},
				{480.294,160.098,10.000,3.730},
				{496.576,162.812,10.000,4.000},
				{512.857,162.812,10.000,4.280},
				{529.138,162.812,10.000,4.580},
				{545.419,162.812,10.000,4.880},
				{561.700,162.812,10.000,5.190},
				{577.981,162.812,10.000,5.520},
				{594.263,162.812,10.000,5.860},
				{610.544,162.812,10.000,6.220},
				{626.825,162.812,10.000,6.580},
				{643.106,162.812,10.000,6.960},
				{659.387,162.812,10.000,7.360},
				{675.669,162.812,10.000,7.770},
				{691.950,162.812,10.000,8.190},
				{708.231,162.812,10.000,8.630},
				{724.512,162.812,10.000,9.080},
				{740.793,162.812,10.000,9.550},
				{757.074,162.812,10.000,10.040},
				{773.356,162.812,10.000,10.540},
				{789.637,162.812,10.000,11.060},
				{805.918,162.812,10.000,11.600},
				{822.199,162.812,10.000,12.160},
				{838.480,162.812,10.000,12.730},
				{854.761,162.812,10.000,13.330},
				{871.043,162.812,10.000,13.940},
				{887.324,162.812,10.000,14.580},
				{903.605,162.812,10.000,15.230},
				{919.886,162.812,10.000,15.910},
				{936.167,162.812,10.000,16.610},
				{952.448,162.812,10.000,17.330},
				{968.730,162.812,10.000,18.070},
				{985.011,162.812,10.000,18.830},
				{1001.292,162.812,10.000,19.620},
				{1017.573,162.812,10.000,20.430},
				{1033.854,162.812,10.000,21.270},
				{1050.135,162.812,10.000,22.130},
				{1066.417,162.812,10.000,23.010},
				{1082.698,162.812,10.000,23.920},
				{1098.979,162.812,10.000,24.850},
				{1115.260,162.812,10.000,25.810},
				{1131.541,162.812,10.000,26.780},
				{1147.822,162.812,10.000,27.780},
				{1164.104,162.812,10.000,28.810},
				{1180.385,162.812,10.000,29.850},
				{1196.666,162.812,10.000,30.920},
				{1212.947,162.812,10.000,32.010},
				{1229.228,162.812,10.000,33.120},
				{1245.509,162.812,10.000,34.240},
				{1261.791,162.812,10.000,35.380},
				{1278.072,162.812,10.000,36.540},
				{1294.353,162.812,10.000,37.710},
				{1310.634,162.812,10.000,38.890},
				{1326.915,162.812,10.000,40.090},
				{1343.196,162.812,10.000,41.290},
				{1359.478,162.812,10.000,42.500},
				{1375.759,162.812,10.000,43.710},
				{1392.040,162.812,10.000,44.920},
				{1408.321,162.812,10.000,46.140},
				{1424.602,162.812,10.000,47.350},
				{1440.883,162.812,10.000,48.560},
				{1457.165,162.812,10.000,49.760},
				{1473.446,162.812,10.000,50.960},
				{1489.727,162.812,10.000,52.140},
				{1506.008,162.812,10.000,53.320},
				{1522.289,162.812,10.000,54.470},
				{1538.570,162.812,10.000,55.620},
				{1554.852,162.812,10.000,56.750},
				{1571.133,162.812,10.000,57.850},
				{1587.414,162.812,10.000,58.940},
				{1603.695,162.812,10.000,60.010},
				{1619.976,162.812,10.000,61.060},
				{1636.257,162.812,10.000,62.090},
				{1652.539,162.812,10.000,63.090},
				{1668.820,162.812,10.000,64.070},
				{1685.101,162.812,10.000,65.030},
				{1701.382,162.812,10.000,65.960},
				{1717.663,162.812,10.000,66.870},
				{1733.944,162.812,10.000,67.760},
				{1750.226,162.812,10.000,68.620},
				{1766.507,162.812,10.000,69.460},
				{1782.788,162.812,10.000,70.270},
				{1799.069,162.812,10.000,71.060},
				{1815.350,162.812,10.000,71.830},
				{1831.631,162.812,10.000,72.580},
				{1847.913,162.812,10.000,73.300},
				{1864.194,162.812,10.000,74.000},
				{1880.475,162.812,10.000,74.680},
				{1896.756,162.812,10.000,75.340},
				{1913.037,162.812,10.000,75.970},
				{1929.318,162.812,10.000,76.590},
				{1945.600,162.812,10.000,77.190},
				{1961.881,162.812,10.000,77.770},
				{1978.162,162.812,10.000,78.320},
				{1994.443,162.812,10.000,78.860},
				{2010.724,162.812,10.000,79.390},
				{2027.006,162.812,10.000,79.890},
				{2043.287,162.812,10.000,80.380},
				{2059.568,162.812,10.000,80.850},
				{2075.849,162.812,10.000,81.310},
				{2092.130,162.812,10.000,81.750},
				{2108.411,162.812,10.000,82.170},
				{2124.693,162.812,10.000,82.580},
				{2140.974,162.812,10.000,82.980},
				{2157.255,162.812,10.000,83.360},
				{2173.536,162.812,10.000,83.730},
				{2189.817,162.812,10.000,84.080},
				{2206.098,162.812,10.000,84.430},
				{2222.380,162.812,10.000,84.760},
				{2238.661,162.812,10.000,85.070},
				{2254.942,162.812,10.000,85.380},
				{2271.223,162.812,10.000,85.670},
				{2287.504,162.812,10.000,85.950},
				{2303.785,162.812,10.000,86.220},
				{2320.067,162.812,10.000,86.490},
				{2336.076,160.098,10.000,86.730},
				{2351.815,157.385,10.000,86.960},
				{2367.282,154.671,10.000,87.180},
				{2382.478,151.958,10.000,87.390},
				{2397.402,149.244,10.000,87.580},
				{2412.055,146.531,10.000,87.760},
				{2426.437,143.817,10.000,87.930},
				{2440.547,141.103,10.000,88.090},
				{2454.386,138.390,10.000,88.240},
				{2467.954,135.676,10.000,88.380},
				{2481.250,132.963,10.000,88.520},
				{2494.275,130.249,10.000,88.640},
				{2507.029,127.536,10.000,88.760},
				{2519.511,124.822,10.000,88.860},
				{2531.722,122.109,10.000,88.970},
				{2543.661,119.395,10.000,89.060},
				{2555.329,116.682,10.000,89.150},
				{2566.726,113.968,10.000,89.230},
				{2577.852,111.255,10.000,89.300},
				{2588.706,108.541,10.000,89.370},
				{2599.289,105.828,10.000,89.440},
				{2609.600,103.114,10.000,89.500},
				{2619.640,100.401,10.000,89.550},
				{2629.409,97.687,10.000,89.600},
				{2638.906,94.973,10.000,89.650},
				{2648.132,92.260,10.000,89.690},
				{2657.087,89.546,10.000,89.720},
				{2665.770,86.833,10.000,89.760},
				{2674.182,84.119,10.000,89.790},
				{2682.323,81.406,10.000,89.820},
				{2690.192,78.692,10.000,89.840},
				{2697.790,75.979,10.000,89.860},
				{2705.116,73.265,10.000,89.880},
				{2712.171,70.552,10.000,89.900},
				{2718.955,67.838,10.000,89.920},
				{2725.468,65.125,10.000,89.930},
				{2731.709,62.411,10.000,89.940},
				{2737.679,59.698,10.000,89.950},
				{2743.377,56.984,10.000,89.960},
				{2748.804,54.271,10.000,89.970},
				{2753.960,51.557,10.000,89.970},
				{2758.844,48.844,10.000,89.980},
				{2763.457,46.130,10.000,89.980},
				{2767.799,43.416,10.000,89.980},
				{2771.869,40.703,10.000,89.990},
				{2775.668,37.989,10.000,89.990},
				{2779.195,35.276,10.000,89.990},
				{2782.452,32.562,10.000,89.990},
				{2785.437,29.849,10.000,89.990},
				{2788.150,27.135,10.000,89.990},
				{2790.592,24.422,10.000,89.990},
				{2792.763,21.708,10.000,89.990},
				{2794.663,18.995,10.000,89.990},
				{2796.291,16.281,10.000,89.990},
				{2797.647,13.568,10.000,89.990},
				{2798.733,10.854,10.000,89.990},
				{2799.547,8.141,10.000,89.990},
				{2800.090,5.427,10.000,89.990}		};

}