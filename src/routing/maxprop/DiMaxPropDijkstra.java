/*
 * Copyright 2010 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details.
 */
package routing.maxprop;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * DiMaxProp Dijkstra shortest path computation.
 *
 * Implements Algorithm 1: DiMaxProp path cost computation at carrier A
 *
 * Require: Locally known contact graph G = (V, E) at carrier A; destination D; distance weight lambda
 * Ensure: Minimum DiMaxProp cost, and next hop neighbor, from A to D
 *
 *  1: cost[A] <- 0
 *  2: for each node u in V \ {A} do
 *  3:   cost[u] <- infinity
 *  4: end for
 *  5: visited <- empty
 *  6: Q <- priority queue initialized with (A, 0)
 *  7: while Q is not empty do
 *  8:   u <- node in Q with smallest cost[u]
 *  9:   remove u from Q
 * 10:   if u in visited then
 * 11:     continue
 * 12:   end if
 * 13:   add u to visited
 * 14:   for each neighbor v of u in E do
 * 15:     if distance estimate exists for edge (u, v) then
 * 16:       lambda_uv <- lambda
 * 17:     else
 * 18:       lambda_uv <- 0
 * 19:     end if
 * 20:     w <- (1 - f(u, v)) + lambda_uv * dnorm(u, v)
 * 21:     if cost[u] + w < cost[v] then
 * 22:       cost[v] <- cost[u] + w
 * 23:       prev[v] <- u
 * 24:       insert or update (v, cost[v]) in Q
 * 25:     end if
 * 26:   end for
 * 27: end while
 * 28: return cost[D] and the neighbor of A on the path traced back from prev[D]
 */
public class DiMaxPropDijkstra {
	/** Value for infinite distance / cost */
	public static final Double INFINITY = Double.MAX_VALUE;

	/** Mapping of hosts' addresses to their meeting probability sets */
	private Map<Integer, MeetingProbabilitySet> allProbs;

	/**
	 * Interface to supply distance estimates and normalized distances between nodes.
	 */
	public interface DistanceEstimator {
		/**
		 * Checks if a distance estimate exists for edge (u, v).
		 * @param u Source node address
		 * @param v Target node address
		 * @return true if distance estimate exists, false otherwise
		 */
		boolean hasDistanceEstimate(int u, int v);

		/**
		 * Returns the normalized distance dnorm(u, v) in range [0, 1].
		 * @param u Source node address
		 * @param v Target node address
		 * @return normalized distance between u and v
		 */
		double getNormalizedDistance(int u, int v);

		/**
		 * Returns raw estimated distance in meters.
		 * @param u Source node address
		 * @param v Target node address
		 * @return distance in meters, or -1.0 if not estimated
		 */
		double getRawDistance(int u, int v);
	}

	/**
	 * Result holding the minimum DiMaxProp cost, next hop neighbor, full path, and step log.
	 */
	public static class PathResult {
		private final double cost;
		private final Integer nextHop;
		private final List<Integer> fullPath;
		private final List<String> computationLog;

		public PathResult(double cost, Integer nextHop) {
			this(cost, nextHop, Collections.<Integer>emptyList(), Collections.<String>emptyList());
		}

		public PathResult(double cost, Integer nextHop, List<Integer> fullPath, List<String> computationLog) {
			this.cost = cost;
			this.nextHop = nextHop;
			this.fullPath = fullPath != null ? fullPath : Collections.<Integer>emptyList();
			this.computationLog = computationLog != null ? computationLog : Collections.<String>emptyList();
		}

		public double getCost() {
			return cost;
		}

		public Integer getNextHop() {
			return nextHop;
		}

		public List<Integer> getFullPath() {
			return fullPath;
		}

		public List<String> getComputationLog() {
			return computationLog;
		}

		@Override
		public String toString() {
			return "PathResult[cost=" + cost + ", nextHop=" + nextHop + ", path=" + fullPath + "]";
		}
	}

	/**
	 * Helper class for PriorityQueue entries: (node, cost).
	 */
	private static class QueueNode implements Comparable<QueueNode> {
		final Integer node;
		final double cost;

		QueueNode(Integer node, double cost) {
			this.node = node;
			this.cost = cost;
		}

		@Override
		public int compareTo(QueueNode o) {
			int cmp = Double.compare(this.cost, o.cost);
			if (cmp != 0) {
				return cmp;
			}
			return this.node.compareTo(o.node);
		}
	}

	/**
	 * Constructor.
	 * @param allProbs A reference to the mapping of known hosts' meeting probability sets
	 */
	public DiMaxPropDijkstra(Map<Integer, MeetingProbabilitySet> allProbs) {
		this.allProbs = allProbs;
	}

	/**
	 * Executes Algorithm 1: DiMaxProp path cost computation at carrier A for destination D.
	 *
	 * @param carrierA Source carrier node A
	 * @param destinationD Target destination node D
	 * @param lambda Distance weight lambda
	 * @param estimator Distance estimator providing dnorm and existence checks
	 * @return PathResult containing minimum cost to D and the next hop neighbor of A
	 */
	public PathResult computePath(Integer carrierA, Integer destinationD,
			double lambda, DistanceEstimator estimator) {
		if (carrierA == null || destinationD == null) {
			return new PathResult(INFINITY, null);
		}
		if (carrierA.equals(destinationD)) {
			return new PathResult(0.0, null, Collections.singletonList(carrierA),
					Collections.singletonList("Carrier A is the destination"));
		}

		List<String> log = new ArrayList<String>();
		log.add(String.format("Algorithm 1: Path computation at Carrier %d -> Destination %d (lambda = %.2f)",
				carrierA, destinationD, lambda));

		// Line 1: cost[A] <- 0
		// Line 2-4: for each node u in V \ {A} do cost[u] <- infinity
		Map<Integer, Double> cost = new HashMap<Integer, Double>();
		Map<Integer, Integer> prev = new HashMap<Integer, Integer>();
		cost.put(carrierA, 0.0);

		// Line 5: visited <- empty
		Set<Integer> visited = new HashSet<Integer>();

		// Line 6: Q <- priority queue initialized with (A, 0)
		PriorityQueue<QueueNode> q = new PriorityQueue<QueueNode>();
		q.add(new QueueNode(carrierA, 0.0));

		// Line 7: while Q is not empty do
		while (!q.isEmpty()) {
			// Line 8: u <- node in Q with smallest cost[u]
			// Line 9: remove u from Q
			QueueNode qNode = q.poll();
			Integer u = qNode.node;

			// Line 10: if u in visited then
			// Line 11:   continue
			// Line 12: end if
			if (visited.contains(u)) {
				continue;
			}

			// Line 13: add u to visited
			visited.add(u);

			if (u.equals(destinationD)) {
				break;
			}

			// Line 14: for each neighbor v of u in E do
			if (!this.allProbs.containsKey(u)) {
				continue;
			}

			MeetingProbabilitySet mps = this.allProbs.get(u);
			Map<Integer, Double> neighbors = mps.getAllProbs();

			for (Map.Entry<Integer, Double> entry : neighbors.entrySet()) {
				Integer v = entry.getKey();
				if (visited.contains(v)) {
					continue;
				}

				// Line 15: if distance estimate exists for edge (u, v) then
				// Line 16:   lambda_uv <- lambda
				// Line 17: else
				// Line 18:   lambda_uv <- 0
				// Line 19: end if
				double lambda_uv;
				double dnorm = 0.0;
				double rawDist = (estimator != null) ? estimator.getRawDistance(u, v) : -1.0;
				if (estimator != null && estimator.hasDistanceEstimate(u, v)) {
					lambda_uv = lambda;
					dnorm = estimator.getNormalizedDistance(u, v);
				} else {
					lambda_uv = 0.0;
				}

				// Line 20: w <- (1 - f(u, v)) + lambda_uv * dnorm(u, v)
				double f_uv = entry.getValue();
				double w = (1.0 - f_uv) + (lambda_uv * dnorm);

				// Line 21: if cost[u] + w < cost[v] then
				double costU = cost.get(u);
				double currentCostV = cost.containsKey(v) ? cost.get(v) : INFINITY;
				double newCostV = costU + w;

				if (newCostV < currentCostV) {
					// Line 22: cost[v] <- cost[u] + w
					cost.put(v, newCostV);
					// Line 23: prev[v] <- u
					prev.put(v, u);
					// Line 24: insert or update (v, cost[v]) in Q
					q.add(new QueueNode(v, newCostV));

					log.add(String.format("  Relax (%d -> %d): f=%.4f, dist=%.1fm, dnorm=%.4f, lambda_uv=%.2f => w=%.4f | new cost[%d]=%.4f",
							u, v, f_uv, rawDist, dnorm, lambda_uv, w, v, newCostV));
				}
			}
		}

		// Line 28: return cost[D] and the neighbor of A on the path traced back from prev[D]
		double costD = cost.containsKey(destinationD) ? cost.get(destinationD) : INFINITY;
		List<Integer> fullPath = buildFullPath(carrierA, destinationD, prev, costD);
		Integer nextHop = traceNextHop(carrierA, destinationD, prev, costD);

		log.add(String.format("Result: cost[%d] = %.4f | Full path: %s | Next hop neighbor: %s",
				destinationD, costD, fullPath, (nextHop != null ? nextHop.toString() : "NONE")));

		return new PathResult(costD, nextHop, fullPath, log);
	}

	/**
	 * Computes DiMaxProp shortest paths to all requested destination nodes in a single search.
	 */
	public Map<Integer, PathResult> computePathsToAll(Integer carrierA,
			Set<Integer> destinations, double lambda, DistanceEstimator estimator) {
		Map<Integer, PathResult> results = new HashMap<Integer, PathResult>();
		if (carrierA == null || destinations == null || destinations.isEmpty()) {
			return results;
		}

		Map<Integer, Double> cost = new HashMap<Integer, Double>();
		Map<Integer, Integer> prev = new HashMap<Integer, Integer>();
		cost.put(carrierA, 0.0);

		Set<Integer> visited = new HashSet<Integer>();
		PriorityQueue<QueueNode> q = new PriorityQueue<QueueNode>();
		q.add(new QueueNode(carrierA, 0.0));

		int remainingDestinations = destinations.size();

		while (!q.isEmpty()) {
			QueueNode qNode = q.poll();
			Integer u = qNode.node;

			if (visited.contains(u)) {
				continue;
			}
			visited.add(u);

			if (destinations.contains(u)) {
				remainingDestinations--;
				if (remainingDestinations <= 0) {
					break;
				}
			}

			if (!this.allProbs.containsKey(u)) {
				continue;
			}

			MeetingProbabilitySet mps = this.allProbs.get(u);
			Map<Integer, Double> neighbors = mps.getAllProbs();

			for (Map.Entry<Integer, Double> entry : neighbors.entrySet()) {
				Integer v = entry.getKey();
				if (visited.contains(v)) {
					continue;
				}

				double lambda_uv;
				double dnorm = 0.0;
				if (estimator != null && estimator.hasDistanceEstimate(u, v)) {
					lambda_uv = lambda;
					dnorm = estimator.getNormalizedDistance(u, v);
				} else {
					lambda_uv = 0.0;
				}

				double f_uv = entry.getValue();
				double w = (1.0 - f_uv) + (lambda_uv * dnorm);

				double costU = cost.get(u);
				double currentCostV = cost.containsKey(v) ? cost.get(v) : INFINITY;
				double newCostV = costU + w;

				if (newCostV < currentCostV) {
					cost.put(v, newCostV);
					prev.put(v, u);
					q.add(new QueueNode(v, newCostV));
				}
			}
		}

		for (Integer d : destinations) {
			if (d.equals(carrierA)) {
				results.put(d, new PathResult(0.0, null, Collections.singletonList(carrierA), null));
			} else {
				double costD = cost.containsKey(d) ? cost.get(d) : INFINITY;
				Integer nextHop = traceNextHop(carrierA, d, prev, costD);
				List<Integer> fullPath = buildFullPath(carrierA, d, prev, costD);
				results.put(d, new PathResult(costD, nextHop, fullPath, null));
			}
		}

		return results;
	}

	/**
	 * Calculates total costs to the given set of target nodes.
	 */
	public Map<Integer, Double> getCosts(Integer from, Set<Integer> to,
			double lambda, DistanceEstimator estimator) {
		Map<Integer, PathResult> paths = computePathsToAll(from, to, lambda, estimator);
		Map<Integer, Double> costs = new HashMap<Integer, Double>();
		for (Map.Entry<Integer, PathResult> e : paths.entrySet()) {
			costs.put(e.getKey(), e.getValue().getCost());
		}
		return costs;
	}

	private List<Integer> buildFullPath(Integer carrierA, Integer destinationD,
			Map<Integer, Integer> prev, double costD) {
		List<Integer> path = new ArrayList<Integer>();
		if (costD >= INFINITY || !prev.containsKey(destinationD)) {
			return path;
		}

		Integer curr = destinationD;
		Set<Integer> loopGuard = new HashSet<Integer>();
		while (curr != null) {
			if (!loopGuard.add(curr)) {
				return Collections.emptyList();
			}
			path.add(0, curr);
			if (curr.equals(carrierA)) {
				break;
			}
			curr = prev.get(curr);
		}
		return path;
	}

	/**
	 * Traces back the shortest path from destination D to carrier A to find the next hop neighbor of A.
	 */
	private Integer traceNextHop(Integer carrierA, Integer destinationD,
			Map<Integer, Integer> prev, double costD) {
		if (costD >= INFINITY || !prev.containsKey(destinationD)) {
			return null;
		}

		Integer curr = destinationD;
		Integer nextHop = null;
		Set<Integer> loopGuard = new HashSet<Integer>();

		while (curr != null && !curr.equals(carrierA)) {
			if (!loopGuard.add(curr)) {
				return null;
			}
			Integer p = prev.get(curr);
			if (p == null) {
				return null;
			}
			if (p.equals(carrierA)) {
				nextHop = curr;
				break;
			}
			curr = p;
		}
		return nextHop;
	}
}
