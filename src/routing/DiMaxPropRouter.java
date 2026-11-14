/*
 * Copyright 2010 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details.
 */
package routing;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import routing.maxprop.DiMaxPropDijkstra;
import routing.maxprop.DiMaxPropDijkstra.DistanceEstimator;
import routing.maxprop.DiMaxPropDijkstra.PathResult;
import routing.maxprop.MeetingProbabilitySet;
import routing.util.RoutingInfo;
import util.Tuple;
import core.Connection;
import core.Coord;
import core.DTNHost;
import core.Message;
import movement.MovementModel;
import core.Settings;
import core.SimClock;
import core.SimScenario;

/**
 * Implementation of DiMaxProp router for Delay-Tolerant Networks.
 *
 * DiMaxProp extends MaxProp by incorporating geographic distance estimates
 * into Dijkstra's shortest path computation:
 *
 * Algorithm 1: DiMaxProp path cost computation at carrier A
 *   w = (1 - f(u, v)) + lambda_uv * dnorm(u, v)
 *
 * where lambda_uv = lambda if a distance estimate exists for edge (u, v),
 * and lambda_uv = 0 otherwise.
 */
public class DiMaxPropRouter extends ActiveRouter implements DistanceEstimator {
	/** Router's setting namespace ({@value}) */
	public static final String DIMAXPROP_NS = "DiMaxPropRouter";

	/** Distance weight lambda setting id ({@value}) */
	public static final String LAMBDA_S = "lambda";
	/** Default value for distance weight lambda ({@value}) */
	public static final double DEFAULT_LAMBDA = 0.5;

	/** Meeting probability set maximum size setting id ({@value}) */
	public static final String PROB_SET_MAX_SIZE_S = "probSetMaxSize";
	/** Default value for the meeting probability set maximum size ({@value}) */
	public static final int DEFAULT_PROB_SET_MAX_SIZE = 50;

	/** Alpha parameter string ({@value}) */
	public static final String ALPHA_S = "alpha";
	/** Default value for alpha ({@value}) */
	public static final double DEFAULT_ALPHA = 1.0;

	/** Maximum world distance for normalization setting id ({@value}) */
	public static final String MAX_DISTANCE_S = "maxDistance";

	/** Strict next-hop forwarding mode setting id ({@value}) */
	public static final String STRICT_NEXT_HOP_S = "strictNextHop";

	/** Oracle distance mode setting id ({@value}) */
	public static final String USE_ORACLE_DISTANCE_S = "useOracleDistance";

	/** Over how many samples the average number of bytes transferred is taken */
	public static final int BYTES_TRANSFERRED_AVG_SAMPLES = 10;

	/* Router parameters */
	private static int probSetMaxSize;
	private double alpha;
	private double lambda;
	private double maxDistance;
	private boolean strictNextHop;
	private boolean useOracleDistance;

	/* State for meeting probabilities and Dijkstra */
	private MeetingProbabilitySet probs;
	private Map<Integer, MeetingProbabilitySet> allProbs;
	private DiMaxPropDijkstra dijkstra;

	/* State for distance and location estimates */
	private Map<Integer, Coord> knownLocations;
	private Map<Integer, Double> locationTimestamps;
	private Map<Integer, Map<Integer, Double>> edgeDistances;

	/* Cached Dijkstra results */
	private Map<Integer, PathResult> cachedResults;
	private DTNHost lastCostFrom;

	/* Message accounting */
	private Set<String> ackedMessageIds;
	private Map<DTNHost, Set<String>> sentMessages;

	/* Transfer rate estimation */
	private int[] avgSamples;
	private int nextSampleIndex = 0;
	private int avgTransferredBytes = 0;

	/**
	 * Constructor. Creates a prototype router based on Settings.
	 * @param s Settings object
	 */
	public DiMaxPropRouter(Settings s) {
		super(s);

		// Read lambda
		if (s.contains(LAMBDA_S)) {
			this.lambda = s.getDouble(LAMBDA_S);
		} else {
			Settings ns = new Settings(DIMAXPROP_NS);
			if (ns.contains(LAMBDA_S)) {
				this.lambda = ns.getDouble(LAMBDA_S);
			} else {
				this.lambda = DEFAULT_LAMBDA;
			}
		}

		// Read alpha
		if (s.contains(ALPHA_S)) {
			this.alpha = s.getDouble(ALPHA_S);
		} else {
			Settings ns = new Settings(DIMAXPROP_NS);
			if (ns.contains(ALPHA_S)) {
				this.alpha = ns.getDouble(ALPHA_S);
			} else {
				this.alpha = DEFAULT_ALPHA;
			}
		}

		// Read probSetMaxSize
		if (s.contains(PROB_SET_MAX_SIZE_S)) {
			probSetMaxSize = s.getInt(PROB_SET_MAX_SIZE_S);
		} else {
			Settings ns = new Settings(DIMAXPROP_NS);
			if (ns.contains(PROB_SET_MAX_SIZE_S)) {
				probSetMaxSize = ns.getInt(PROB_SET_MAX_SIZE_S);
			} else {
				probSetMaxSize = DEFAULT_PROB_SET_MAX_SIZE;
			}
		}

		// Read maxDistance
		if (s.contains(MAX_DISTANCE_S)) {
			this.maxDistance = s.getDouble(MAX_DISTANCE_S);
		} else {
			Settings ns = new Settings(DIMAXPROP_NS);
			if (ns.contains(MAX_DISTANCE_S)) {
				this.maxDistance = ns.getDouble(MAX_DISTANCE_S);
			} else {
				this.maxDistance = -1.0; // Auto-calculate later
			}
		}

		// Read strictNextHop
		if (s.contains(STRICT_NEXT_HOP_S)) {
			this.strictNextHop = s.getBoolean(STRICT_NEXT_HOP_S);
		} else {
			Settings ns = new Settings(DIMAXPROP_NS);
			if (ns.contains(STRICT_NEXT_HOP_S)) {
				this.strictNextHop = ns.getBoolean(STRICT_NEXT_HOP_S);
			} else {
				this.strictNextHop = false;
			}
		}

		// Read useOracleDistance
		if (s.contains(USE_ORACLE_DISTANCE_S)) {
			this.useOracleDistance = s.getBoolean(USE_ORACLE_DISTANCE_S);
		} else {
			Settings ns = new Settings(DIMAXPROP_NS);
			if (ns.contains(USE_ORACLE_DISTANCE_S)) {
				this.useOracleDistance = ns.getBoolean(USE_ORACLE_DISTANCE_S);
			} else {
				this.useOracleDistance = false;
			}
		}
	}

	/**
	 * Copy constructor. Replicates prototype for each host.
	 * @param r Prototype router
	 */
	protected DiMaxPropRouter(DiMaxPropRouter r) {
		super(r);
		this.alpha = r.alpha;
		this.lambda = r.lambda;
		this.maxDistance = r.maxDistance;
		this.strictNextHop = r.strictNextHop;
		this.useOracleDistance = r.useOracleDistance;

		this.probs = new MeetingProbabilitySet(probSetMaxSize, this.alpha);
		this.allProbs = new HashMap<Integer, MeetingProbabilitySet>();
		this.dijkstra = new DiMaxPropDijkstra(this.allProbs);

		this.knownLocations = new HashMap<Integer, Coord>();
		this.locationTimestamps = new HashMap<Integer, Double>();
		this.edgeDistances = new HashMap<Integer, Map<Integer, Double>>();

		this.ackedMessageIds = new HashSet<String>();
		this.avgSamples = new int[BYTES_TRANSFERRED_AVG_SAMPLES];
		this.sentMessages = new HashMap<DTNHost, Set<String>>();
	}

	/**
	 * Returns maximum distance used for distance normalization.
	 * If not set explicitly, auto-calculates from world size.
	 */
	public double getMaxDistance() {
		if (this.maxDistance > 0.0) {
			return this.maxDistance;
		}

		try {
			if (SimScenario.getInstance() != null && SimScenario.getInstance().getWorld() != null) {
				double wx = SimScenario.getInstance().getWorld().getSizeX();
				double wy = SimScenario.getInstance().getWorld().getSizeY();
				this.maxDistance = Math.hypot(wx, wy);
				return this.maxDistance;
			}
		} catch (Exception e) {
			// Fallback if scenario world not yet ready
		}

		try {
			Settings mSettings = new Settings(MovementModel.MOVEMENT_MODEL_NS);
			if (mSettings.contains(MovementModel.WORLD_SIZE)) {
				int[] ws = mSettings.getCsvInts(MovementModel.WORLD_SIZE, 2);
				this.maxDistance = Math.hypot(ws[0], ws[1]);
				return this.maxDistance;
			}
		} catch (Exception e) {
			// Fallback
		}

		this.maxDistance = 5000.0;
		return this.maxDistance;
	}

	@Override
	public void changedConnection(Connection con) {
		super.changedConnection(con);

		if (con.isUp()) {
			this.cachedResults = null; // Invalidate cost cache

			if (con.isInitiator(getHost())) {
				DTNHost otherHost = con.getOtherNode(getHost());
				MessageRouter mRouter = otherHost.getRouter();

				assert mRouter instanceof DiMaxPropRouter :
					"DiMaxProp only works with other routers of same type";
				DiMaxPropRouter otherRouter = (DiMaxPropRouter) mRouter;

				/* Exchange ACKed messages */
				this.ackedMessageIds.addAll(otherRouter.ackedMessageIds);
				otherRouter.ackedMessageIds.addAll(this.ackedMessageIds);
				deleteAckedMessages();
				otherRouter.deleteAckedMessages();

				/* Update meeting probabilities */
				this.probs.updateMeetingProbFor(otherHost.getAddress());
				otherRouter.probs.updateMeetingProbFor(getHost().getAddress());

				/* Exchange transitive probabilities */
				this.updateTransitiveProbs(otherRouter.allProbs);
				otherRouter.updateTransitiveProbs(this.allProbs);
				this.allProbs.put(otherHost.getAddress(), otherRouter.probs.replicate());
				otherRouter.allProbs.put(getHost().getAddress(), this.probs.replicate());

				/* Record encounter location and distance */
				double now = SimClock.getTime();
				Coord myLoc = getHost().getLocation().clone();
				Coord othLoc = otherHost.getLocation().clone();
				double encDist = myLoc.distance(othLoc);

				this.recordLocation(getHost().getAddress(), myLoc, now);
				this.recordLocation(otherHost.getAddress(), othLoc, now);
				this.recordEdgeDistance(getHost().getAddress(), otherHost.getAddress(), encDist);

				otherRouter.recordLocation(otherHost.getAddress(), othLoc, now);
				otherRouter.recordLocation(getHost().getAddress(), myLoc, now);
				otherRouter.recordEdgeDistance(otherHost.getAddress(), getHost().getAddress(), encDist);

				/* Exchange known locations and edge distances */
				this.updateTransitiveLocations(otherRouter.knownLocations, otherRouter.locationTimestamps);
				otherRouter.updateTransitiveLocations(this.knownLocations, this.locationTimestamps);

				this.updateTransitiveEdgeDistances(otherRouter.edgeDistances);
				otherRouter.updateTransitiveEdgeDistances(this.edgeDistances);
			}
		} else {
			updateTransferredBytesAvg(con.getTotalBytesTransferred());
		}
	}

	/**
	 * Records a known location with timestamp.
	 */
	public void recordLocation(int hostAddress, Coord loc, double time) {
		this.knownLocations.put(hostAddress, loc.clone());
		this.locationTimestamps.put(hostAddress, time);
	}

	/**
	 * Records an edge distance between two hosts.
	 */
	public void recordEdgeDistance(int u, int v, double dist) {
		Map<Integer, Double> uMap = this.edgeDistances.get(u);
		if (uMap == null) {
			uMap = new HashMap<Integer, Double>();
			this.edgeDistances.put(u, uMap);
		}
		uMap.put(v, dist);

		Map<Integer, Double> vMap = this.edgeDistances.get(v);
		if (vMap == null) {
			vMap = new HashMap<Integer, Double>();
			this.edgeDistances.put(v, vMap);
		}
		vMap.put(u, dist);
	}

	/**
	 * Exchanges and merges known location information based on timestamp freshness.
	 */
	private void updateTransitiveLocations(Map<Integer, Coord> otherLocs,
			Map<Integer, Double> otherTimes) {
		for (Map.Entry<Integer, Coord> entry : otherLocs.entrySet()) {
			int addr = entry.getKey();
			double otherTime = otherTimes.containsKey(addr) ? otherTimes.get(addr) : 0.0;
			double myTime = this.locationTimestamps.containsKey(addr) ?
					this.locationTimestamps.get(addr) : -1.0;

			if (otherTime > myTime) {
				this.knownLocations.put(addr, entry.getValue().clone());
				this.locationTimestamps.put(addr, otherTime);
			}
		}
	}

	/**
	 * Exchanges and merges edge distance estimates.
	 */
	private void updateTransitiveEdgeDistances(Map<Integer, Map<Integer, Double>> otherDistances) {
		for (Map.Entry<Integer, Map<Integer, Double>> entry : otherDistances.entrySet()) {
			int u = entry.getKey();
			Map<Integer, Double> myMap = this.edgeDistances.get(u);
			if (myMap == null) {
				myMap = new HashMap<Integer, Double>();
				this.edgeDistances.put(u, myMap);
			}
			for (Map.Entry<Integer, Double> inner : entry.getValue().entrySet()) {
				myMap.put(inner.getKey(), inner.getValue());
			}
		}
	}

	/**
	 * Updates transitive meeting probabilities.
	 */
	private void updateTransitiveProbs(Map<Integer, MeetingProbabilitySet> p) {
		for (Map.Entry<Integer, MeetingProbabilitySet> e : p.entrySet()) {
			MeetingProbabilitySet myMps = this.allProbs.get(e.getKey());
			if (myMps == null ||
				e.getValue().getLastUpdateTime() > myMps.getLastUpdateTime()) {
				this.allProbs.put(e.getKey(), e.getValue().replicate());
			}
		}
	}

	/**
	 * Deletes ACKed messages from the local buffer.
	 */
	private void deleteAckedMessages() {
		for (String id : this.ackedMessageIds) {
			if (this.hasMessage(id) && !isSending(id)) {
				this.deleteMessage(id, false);
			}
		}
	}

	@Override
	public Message messageTransferred(String id, DTNHost from) {
		this.cachedResults = null;
		Message m = super.messageTransferred(id, from);
		if (isDeliveredMessage(m)) {
			this.ackedMessageIds.add(id);
		}
		return m;
	}

	@Override
	protected void transferDone(Connection con) {
		Message m = con.getMessage();
		String id = m.getId();
		DTNHost recipient = con.getOtherNode(getHost());
		Set<String> sentMsgIds = this.sentMessages.get(recipient);

		if (m.getTo() == recipient) {
			this.ackedMessageIds.add(m.getId());
			this.deleteMessage(m.getId(), false);
		}

		if (sentMsgIds == null) {
			sentMsgIds = new HashSet<String>();
			this.sentMessages.put(recipient, sentMsgIds);
		}
		sentMsgIds.add(id);
	}

	private void updateTransferredBytesAvg(int newValue) {
		int realCount = 0;
		int sum = 0;

		this.avgSamples[this.nextSampleIndex++] = newValue;
		if (this.nextSampleIndex >= BYTES_TRANSFERRED_AVG_SAMPLES) {
			this.nextSampleIndex = 0;
		}

		for (int i = 0; i < BYTES_TRANSFERRED_AVG_SAMPLES; i++) {
			if (this.avgSamples[i] > 0) {
				realCount++;
				sum += this.avgSamples[i];
			}
		}

		if (realCount > 0) {
			this.avgTransferredBytes = sum / realCount;
		} else {
			this.avgTransferredBytes = 0;
		}
	}

	@Override
	public void update() {
		super.update();
		if (!canStartTransfer() || isTransferring()) {
			return;
		}

		// Try messages directly deliverable to final destination
		if (exchangeDeliverableMessages() != null) {
			return;
		}

		tryOtherMessages();
	}

	/**
	 * Tries to send messages to connected hosts, prioritizing next hop and minimum DiMaxProp cost.
	 */
	private Tuple<Message, Connection> tryOtherMessages() {
		List<Tuple<Message, Connection>> candidateTuples =
			new ArrayList<Tuple<Message, Connection>>();

		Collection<Message> msgCollection = getMessageCollection();
		if (msgCollection.isEmpty() || getConnections().isEmpty()) {
			return null;
		}

		for (Connection con : getConnections()) {
			DTNHost other = con.getOtherNode(getHost());
			ActiveRouter othRouter = (ActiveRouter) other.getRouter();

			if (othRouter.isTransferring()) {
				continue;
			}

			Set<String> sentMsgIds = this.sentMessages.get(other);

			for (Message m : msgCollection) {
				if (othRouter.hasMessage(m.getId()) || m.getHops().contains(other)) {
					continue;
				}
				if (sentMsgIds != null && sentMsgIds.contains(m.getId())) {
					continue;
				}

				// Check next hop guidance from DiMaxProp
				PathResult res = getPathResult(getHost(), m.getTo());
				Integer nextHop = res.getNextHop();

				if (strictNextHop) {
					// Only forward if the connected node is the designated next hop or destination
					if (nextHop == null || !nextHop.equals(other.getAddress())) {
						if (m.getTo() != other) {
							continue;
						}
					}
				}

				candidateTuples.add(new Tuple<Message, Connection>(m, con));
			}
		}

		if (candidateTuples.isEmpty()) {
			return null;
		}

		Collections.sort(candidateTuples, new DiMaxPropTupleComparator(calcThreshold()));
		return tryMessagesForConnected(candidateTuples);
	}

	/**
	 * Computes path result (cost and next-hop) from start host to destination host.
	 */
	public PathResult getPathResult(DTNHost from, DTNHost to) {
		if (this.cachedResults == null || lastCostFrom != from) {
			this.allProbs.put(getHost().getAddress(), this.probs);
			int fromIndex = from.getAddress();

			Set<Integer> toSet = new HashSet<Integer>();
			for (Message m : getMessageCollection()) {
				toSet.add(m.getTo().getAddress());
			}

			this.cachedResults = dijkstra.computePathsToAll(fromIndex, toSet, this.lambda, this);
			this.lastCostFrom = from;
		}

		if (this.cachedResults != null && this.cachedResults.containsKey(to.getAddress())) {
			return this.cachedResults.get(to.getAddress());
		}

		return dijkstra.computePath(from.getAddress(), to.getAddress(), this.lambda, this);
	}

	/**
	 * Returns DiMaxProp delivery cost between two hosts.
	 */
	public double getCost(DTNHost from, DTNHost to) {
		return getPathResult(from, to).getCost();
	}

	/**
	 * Returns path result with full execution log trace for reporting and inspection.
	 */
	public PathResult getPathResultWithTrace(DTNHost from, DTNHost to) {
		return dijkstra.computePath(from.getAddress(), to.getAddress(), this.lambda, this);
	}

	/**
	 * DistanceEstimator interface implementation:
	 * returns raw estimated distance in meters, or -1.0 if not estimated.
	 */
	@Override
	public double getRawDistance(int u, int v) {
		if (this.edgeDistances.containsKey(u) && this.edgeDistances.get(u).containsKey(v)) {
			return this.edgeDistances.get(u).get(v);
		}
		if (this.knownLocations.containsKey(u) && this.knownLocations.containsKey(v)) {
			return this.knownLocations.get(u).distance(this.knownLocations.get(v));
		}
		if (useOracleDistance) {
			try {
				if (SimScenario.getInstance() != null && SimScenario.getInstance().getWorld() != null) {
					DTNHost hu = SimScenario.getInstance().getWorld().getNodeByAddress(u);
					DTNHost hv = SimScenario.getInstance().getWorld().getNodeByAddress(v);
					if (hu != null && hv != null && hu.getLocation() != null && hv.getLocation() != null) {
						return hu.getLocation().distance(hv.getLocation());
					}
				}
			} catch (Exception e) {
				// Ignore
			}
		}
		return -1.0;
	}

	public double getLambda() { return this.lambda; }
	public double getAlpha() { return this.alpha; }
	public boolean isStrictNextHop() { return this.strictNextHop; }
	public MeetingProbabilitySet getProbs() { return this.probs; }
	public Map<Integer, MeetingProbabilitySet> getAllProbs() { return this.allProbs; }
	public Map<Integer, Coord> getKnownLocations() { return this.knownLocations; }

	/**
	 * DistanceEstimator interface implementation:
	 * checks if distance estimate exists for edge (u, v).
	 */
	@Override
	public boolean hasDistanceEstimate(int u, int v) {
		if (this.edgeDistances.containsKey(u) && this.edgeDistances.get(u).containsKey(v)) {
			return true;
		}
		if (this.knownLocations.containsKey(u) && this.knownLocations.containsKey(v)) {
			return true;
		}
		if (useOracleDistance) {
			try {
				if (SimScenario.getInstance() != null && SimScenario.getInstance().getWorld() != null) {
					DTNHost hu = SimScenario.getInstance().getWorld().getNodeByAddress(u);
					DTNHost hv = SimScenario.getInstance().getWorld().getNodeByAddress(v);
					if (hu != null && hv != null && hu.getLocation() != null && hv.getLocation() != null) {
						return true;
					}
				}
			} catch (Exception e) {
				// Ignore
			}
		}
		return false;
	}

	/**
	 * DistanceEstimator interface implementation:
	 * returns normalized distance dnorm(u, v) in range [0, 1].
	 */
	@Override
	public double getNormalizedDistance(int u, int v) {
		double rawDist = 0.0;
		boolean found = false;

		if (this.edgeDistances.containsKey(u) && this.edgeDistances.get(u).containsKey(v)) {
			rawDist = this.edgeDistances.get(u).get(v);
			found = true;
		} else if (this.knownLocations.containsKey(u) && this.knownLocations.containsKey(v)) {
			rawDist = this.knownLocations.get(u).distance(this.knownLocations.get(v));
			found = true;
		} else if (useOracleDistance) {
			try {
				if (SimScenario.getInstance() != null && SimScenario.getInstance().getWorld() != null) {
					DTNHost hu = SimScenario.getInstance().getWorld().getNodeByAddress(u);
					DTNHost hv = SimScenario.getInstance().getWorld().getNodeByAddress(v);
					if (hu != null && hv != null && hu.getLocation() != null && hv.getLocation() != null) {
						rawDist = hu.getLocation().distance(hv.getLocation());
						found = true;
					}
				}
			} catch (Exception e) {
				// Ignore
			}
		}

		if (!found) {
			return 0.0;
		}

		double maxD = getMaxDistance();
		if (maxD <= 0.0) {
			return 0.0;
		}
		return Math.min(1.0, rawDist / maxD);
	}

	/**
	 * Calculates threshold value for priority buffer split.
	 */
	public int calcThreshold() {
		long b = this.getBufferSize();
		long x = this.avgTransferredBytes;
		long p;

		if (x == 0) {
			return 0;
		}

		if (x < b / 2) {
			p = x;
		} else if (b / 2 <= x && x < b) {
			p = Math.min(x, b - x);
		} else {
			return 0;
		}

		ArrayList<Message> msgs = new ArrayList<Message>();
		msgs.addAll(getMessageCollection());
		if (msgs.isEmpty()) {
			return 0;
		}

		Collections.sort(msgs, new Comparator<Message>() {
			public int compare(Message m1, Message m2) {
				return m1.getHopCount() - m2.getHopCount();
			}
		});

		int i = 0;
		for (int n = msgs.size(); i < n && p > 0; i++) {
			p -= msgs.get(i).getSize();
		}

		i--;
		if (i < 0) {
			return 0;
		}

		return msgs.get(i).getHopCount() + 1;
	}

	@Override
	protected Message getNextMessageToRemove(boolean excludeMsgBeingSent) {
		Collection<Message> messages = this.getMessageCollection();
		List<Message> validMessages = new ArrayList<Message>();

		for (Message m : messages) {
			if (excludeMsgBeingSent && isSending(m.getId())) {
				continue;
			}
			validMessages.add(m);
		}

		if (validMessages.isEmpty()) {
			return null;
		}

		Collections.sort(validMessages, new DiMaxPropComparator(this.calcThreshold()));
		return validMessages.get(validMessages.size() - 1);
	}

	/**
	 * Message comparator using DiMaxProp path cost and hop threshold.
	 */
	private class DiMaxPropComparator implements Comparator<Message> {
		private int threshold;
		private DTNHost from1;
		private DTNHost from2;

		public DiMaxPropComparator(int threshold) {
			this.threshold = threshold;
			this.from1 = this.from2 = getHost();
		}

		public DiMaxPropComparator(int threshold, DTNHost from1, DTNHost from2) {
			this.threshold = threshold;
			this.from1 = from1;
			this.from2 = from2;
		}

		@Override
		public int compare(Message msg1, Message msg2) {
			int hopc1 = msg1.getHopCount();
			int hopc2 = msg2.getHopCount();

			if (msg1 == msg2) {
				return 0;
			}

			if (hopc1 < threshold && hopc2 >= threshold) {
				return -1;
			} else if (hopc2 < threshold && hopc1 >= threshold) {
				return 1;
			}

			if (hopc1 < threshold && hopc2 < threshold) {
				return hopc1 - hopc2;
			}

			double p1 = getCost(from1, msg1.getTo());
			double p2 = getCost(from2, msg2.getTo());

			if (Double.compare(p1, p2) == 0) {
				if (hopc1 == hopc2) {
					return compareByQueueMode(msg1, msg2);
				} else {
					return hopc1 - hopc2;
				}
			} else if (p1 < p2) {
				return -1;
			} else {
				return 1;
			}
		}
	}

	/**
	 * Tuple comparator prioritizing next-hop matches and DiMaxProp minimum cost.
	 */
	private class DiMaxPropTupleComparator implements Comparator<Tuple<Message, Connection>> {
		private int threshold;

		public DiMaxPropTupleComparator(int threshold) {
			this.threshold = threshold;
		}

		@Override
		public int compare(Tuple<Message, Connection> tuple1,
				Tuple<Message, Connection> tuple2) {
			Message m1 = tuple1.getKey();
			Message m2 = tuple2.getKey();
			DTNHost receiver1 = tuple1.getValue().getOtherNode(getHost());
			DTNHost receiver2 = tuple2.getValue().getOtherNode(getHost());

			// Priority 1: Check if receiver matches the DiMaxProp next hop
			PathResult res1 = getPathResult(getHost(), m1.getTo());
			PathResult res2 = getPathResult(getHost(), m2.getTo());

			boolean isNextHop1 = res1.getNextHop() != null &&
					res1.getNextHop().equals(receiver1.getAddress());
			boolean isNextHop2 = res2.getNextHop() != null &&
					res2.getNextHop().equals(receiver2.getAddress());

			if (isNextHop1 && !isNextHop2) {
				return -1;
			} else if (!isNextHop1 && isNextHop2) {
				return 1;
			}

			// Priority 2: Use DiMaxProp cost comparison
			DiMaxPropComparator comp = new DiMaxPropComparator(threshold, receiver1, receiver2);
			return comp.compare(m1, m2);
		}
	}

	@Override
	public RoutingInfo getRoutingInfo() {
		RoutingInfo top = super.getRoutingInfo();
		RoutingInfo ri = new RoutingInfo(probs.getAllProbs().size() + " meeting probabilities");

		for (Map.Entry<Integer, Double> e : probs.getAllProbs().entrySet()) {
			ri.addMoreInfo(new RoutingInfo(String.format("host %d : %.6f",
					e.getKey(), e.getValue())));
		}

		top.addMoreInfo(ri);
		top.addMoreInfo(new RoutingInfo(String.format(
				"DiMaxProp [lambda=%.2f, alpha=%.2f, maxDist=%.1f, knownLocs=%d]",
				this.lambda, this.alpha, getMaxDistance(), this.knownLocations.size())));
		top.addMoreInfo(new RoutingInfo("Avg transferred bytes: " + this.avgTransferredBytes));

		return top;
	}

	@Override
	public MessageRouter replicate() {
		return new DiMaxPropRouter(this);
	}
}
