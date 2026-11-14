package routing.maxprop;

import java.util.HashMap;
import java.util.Map;
import routing.maxprop.DiMaxPropDijkstra.DistanceEstimator;
import routing.maxprop.DiMaxPropDijkstra.PathResult;

/**
 * Standalone verification runner for Algorithm 1 DiMaxProp path cost computation.
 */
public class DiMaxPropTestRunner {

	private static void assertTrue(String msg, boolean cond) {
		if (!cond) {
			throw new AssertionError("FAILED: " + msg);
		}
		System.out.println("  PASS: " + msg);
	}

	private static void assertEquals(String msg, double expected, double actual, double delta) {
		if (Math.abs(expected - actual) > delta) {
			throw new AssertionError("FAILED: " + msg + " (expected=" + expected + ", actual=" + actual + ")");
		}
		System.out.println("  PASS: " + msg + " (" + actual + ")");
	}

	private static void assertEquals(String msg, Object expected, Object actual) {
		if (expected == null && actual == null) {
			System.out.println("  PASS: " + msg + " (null)");
			return;
		}
		if (expected == null || !expected.equals(actual)) {
			throw new AssertionError("FAILED: " + msg + " (expected=" + expected + ", actual=" + actual + ")");
		}
		System.out.println("  PASS: " + msg + " (" + actual + ")");
	}

	public static void main(String[] args) {
		System.out.println("==================================================");
		System.out.println("Running DiMaxProp Algorithm 1 Verification Tests");
		System.out.println("==================================================");

		testStandardMaxPropEquivalence();
		testDistanceIntegratedWeightAndNextHopSelection();
		testPartialDistanceEstimates();
		testDirectDestinationAndSelf();
		testUnreachable();

		System.out.println("==================================================");
		System.out.println("All DiMaxProp Algorithm 1 Tests Passed Successfully!");
		System.out.println("==================================================");
	}

	/**
	 * Test 1: When lambda = 0 or no distance estimates exist,
	 * w = 1 - f(u, v), identical to standard MaxProp.
	 */
	private static void testStandardMaxPropEquivalence() {
		System.out.println("\nTest 1: Standard MaxProp Equivalence (lambda = 0 / no distance)");
		Map<Integer, MeetingProbabilitySet> mapping = new HashMap<Integer, MeetingProbabilitySet>();
		for (int i = 0; i < 3; i++) {
			mapping.put(i, new MeetingProbabilitySet(50, 1.0));
		}
		mapping.get(0).updateMeetingProbFor(1, 0.8); // edge 0->1: f=0.8, w=0.2
		mapping.get(1).updateMeetingProbFor(2, 0.5); // edge 1->2: f=0.5, w=0.5
		// Total cost 0->2: 0.2 + 0.5 = 0.7

		DiMaxPropDijkstra dijkstra = new DiMaxPropDijkstra(mapping);
		PathResult res = dijkstra.computePath(0, 2, 0.5, null);

		assertEquals("Path cost 0 -> 2", 0.7, res.getCost(), 0.0001);
		assertEquals("Next hop from 0 to 2", Integer.valueOf(1), res.getNextHop());
	}

	/**
	 * Test 2: When distance estimates exist and lambda > 0,
	 * w = (1 - f(u, v)) + lambda * dnorm(u, v).
	 * Verifies that spatial proximity correctly redirects routing towards closer path!
	 */
	private static void testDistanceIntegratedWeightAndNextHopSelection() {
		System.out.println("\nTest 2: Distance-Integrated Weight & Path/Next-Hop Selection");
		Map<Integer, MeetingProbabilitySet> mapping = new HashMap<Integer, MeetingProbabilitySet>();
		for (int i = 0; i < 4; i++) {
			mapping.put(i, new MeetingProbabilitySet(50, 1.0));
		}

		// Carrier 0 can reach Destination 3 via Node 1 or Node 2
		// Path A: 0 -> 1 -> 3
		// f(0, 1) = 0.6, dnorm = 0.1
		// f(1, 3) = 0.5, dnorm = 0.1
		// Without distance: cost = (1-0.6) + (1-0.5) = 0.4 + 0.5 = 0.9

		// Path B: 0 -> 2 -> 3
		// f(0, 2) = 0.85, dnorm = 0.9
		// f(2, 3) = 0.85, dnorm = 0.9
		// Without distance: cost = (1-0.85) + (1-0.85) = 0.15 + 0.15 = 0.3 (Path B would win)

		mapping.get(0).updateMeetingProbFor(1, 0.6);
		mapping.get(0).updateMeetingProbFor(2, 0.85);
		mapping.get(1).updateMeetingProbFor(3, 0.5);
		mapping.get(2).updateMeetingProbFor(3, 0.85);

		final Map<String, Double> dists = new HashMap<String, Double>();
		dists.put("0-1", 0.1);
		dists.put("1-3", 0.1);
		dists.put("0-2", 0.9);
		dists.put("2-3", 0.9);

		DistanceEstimator estimator = new DistanceEstimator() {
			public boolean hasDistanceEstimate(int u, int v) {
				return dists.containsKey(u + "-" + v);
			}
			public double getNormalizedDistance(int u, int v) {
				return dists.get(u + "-" + v);
			}
			public double getRawDistance(int u, int v) {
				return dists.containsKey(u + "-" + v) ? dists.get(u + "-" + v) * 100.0 : -1.0;
			}
		};

		DiMaxPropDijkstra dijkstra = new DiMaxPropDijkstra(mapping);

		// With lambda = 1.0:
		// Weight Path A: (1 - 0.6 + 1.0*0.1) + (1 - 0.5 + 1.0*0.1) = 0.5 + 0.6 = 1.10
		// Weight Path B: (1 - 0.85 + 1.0*0.9) + (1 - 0.85 + 1.0*0.9) = 1.05 + 1.05 = 2.10
		// DiMaxProp favors Path A due to geographical proximity!
		PathResult res = dijkstra.computePath(0, 3, 1.0, estimator);

		assertEquals("DiMaxProp cost favoring closer Path A", 1.10, res.getCost(), 0.0001);
		assertEquals("Next hop neighbor selected", Integer.valueOf(1), res.getNextHop());
	}

	/**
	 * Test 3: Lines 15-19 of Algorithm 1: selective lambda_uv.
	 * If distance estimate exists, lambda_uv = lambda; else lambda_uv = 0.
	 */
	private static void testPartialDistanceEstimates() {
		System.out.println("\nTest 3: Selective Distance Weight (Algorithm 1 lines 15-19)");
		Map<Integer, MeetingProbabilitySet> mapping = new HashMap<Integer, MeetingProbabilitySet>();
		for (int i = 0; i < 3; i++) {
			mapping.put(i, new MeetingProbabilitySet(50, 1.0));
		}

		mapping.get(0).updateMeetingProbFor(1, 0.7); // has estimate dnorm=0.2
		mapping.get(1).updateMeetingProbFor(2, 0.6); // NO estimate

		DistanceEstimator partialEstimator = new DistanceEstimator() {
			public boolean hasDistanceEstimate(int u, int v) {
				return (u == 0 && v == 1);
			}
			public double getNormalizedDistance(int u, int v) {
				return (u == 0 && v == 1) ? 0.2 : 0.0;
			}
			public double getRawDistance(int u, int v) {
				return (u == 0 && v == 1) ? 20.0 : -1.0;
			}
		};

		DiMaxPropDijkstra dijkstra = new DiMaxPropDijkstra(mapping);
		double lambda = 0.5;
		// Edge (0, 1): (1 - 0.7) + 0.5 * 0.2 = 0.3 + 0.1 = 0.4
		// Edge (1, 2): (1 - 0.6) + 0.0 = 0.4
		// Total: 0.8
		PathResult res = dijkstra.computePath(0, 2, lambda, partialEstimator);

		assertEquals("Path cost with partial estimate", 0.8, res.getCost(), 0.0001);
		assertEquals("Next hop from 0 to 2", Integer.valueOf(1), res.getNextHop());
	}

	/**
	 * Test 4: Direct destination and self cost.
	 */
	private static void testDirectDestinationAndSelf() {
		System.out.println("\nTest 4: Direct Neighbor and Self Destination");
		Map<Integer, MeetingProbabilitySet> mapping = new HashMap<Integer, MeetingProbabilitySet>();
		mapping.put(0, new MeetingProbabilitySet(50, 1.0));
		mapping.put(1, new MeetingProbabilitySet(50, 1.0));
		mapping.get(0).updateMeetingProbFor(1, 0.4);

		DistanceEstimator est = new DistanceEstimator() {
			public boolean hasDistanceEstimate(int u, int v) { return true; }
			public double getNormalizedDistance(int u, int v) { return 0.2; }
			public double getRawDistance(int u, int v) { return 20.0; }
		};

		DiMaxPropDijkstra dijkstra = new DiMaxPropDijkstra(mapping);

		// Self
		PathResult selfRes = dijkstra.computePath(0, 0, 0.5, est);
		assertEquals("Cost to self", 0.0, selfRes.getCost(), 0.0001);
		assertEquals("Next hop to self", null, selfRes.getNextHop());

		// Direct
		PathResult directRes = dijkstra.computePath(0, 1, 0.5, est);
		// (1 - 0.4) + 0.5 * 0.2 = 0.6 + 0.1 = 0.7
		assertEquals("Cost to direct neighbor", 0.7, directRes.getCost(), 0.0001);
		assertEquals("Next hop to direct neighbor", Integer.valueOf(1), directRes.getNextHop());
	}

	/**
	 * Test 5: Unreachable destination.
	 */
	private static void testUnreachable() {
		System.out.println("\nTest 5: Unreachable Destination");
		Map<Integer, MeetingProbabilitySet> mapping = new HashMap<Integer, MeetingProbabilitySet>();
		mapping.put(0, new MeetingProbabilitySet(50, 1.0));
		mapping.put(1, new MeetingProbabilitySet(50, 1.0));
		mapping.get(0).updateMeetingProbFor(1, 0.5);

		DiMaxPropDijkstra dijkstra = new DiMaxPropDijkstra(mapping);
		PathResult res = dijkstra.computePath(0, 99, 0.5, null);

		assertEquals("Cost to unreachable node", DiMaxPropDijkstra.INFINITY, res.getCost(), 0.0001);
		assertEquals("Next hop to unreachable node", null, res.getNextHop());
	}
}

