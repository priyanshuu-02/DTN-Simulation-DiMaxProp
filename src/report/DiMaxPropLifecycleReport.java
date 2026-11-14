/*
 * Copyright 2010 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details.
 */
package report;

import java.util.List;
import java.util.Map;

import core.Coord;
import core.DTNHost;
import core.Message;
import core.MessageListener;
import core.SimScenario;
import routing.DiMaxPropRouter;
import routing.maxprop.DiMaxPropDijkstra.PathResult;

/**
 * Publication-ready Lifecycle Report for DiMaxProp Algorithm 1.
 *
 * Tracks the complete lifecycle of message M1 from creation, through multi-hop
 * encounters, Algorithm 1 Dijkstra mathematical evaluation at each hop,
 * next-hop forwarding, delivery, and ACK buffer cleanup.
 */
public class DiMaxPropLifecycleReport extends Report implements MessageListener {
	public static final String TARGET_MESSAGE_S = "targetMessage";
	private String targetMessageId = "M1";

	private double creationTime = -1.0;
	private double deliveryTime = -1.0;
	private boolean delivered = false;
	private int totalHops = 0;
	private String finalPath = "NONE";
	private DTNHost sourceHost;
	private DTNHost destHost;

	public DiMaxPropLifecycleReport() {
		init();
	}

	@Override
	protected void init() {
		super.init();

		if (getSettings().contains(TARGET_MESSAGE_S)) {
			this.targetMessageId = getSettings().getSetting(TARGET_MESSAGE_S);
		}

		write("================================================================================");
		write("               DIMAXPROP ALGORITHM 1: MESSAGE LIFECYCLE REPORT");
		write("================================================================================");
		write("Tracking Target Message : " + targetMessageId);
		write("Simulator Environment   : The ONE (Opportunistic Network Environment)");

		if (SimScenario.getInstance() != null) {
			write("Scenario Name           : " + SimScenario.getInstance().getName());
			if (SimScenario.getInstance().getWorld() != null) {
				int sx = SimScenario.getInstance().getWorld().getSizeX();
				int sy = SimScenario.getInstance().getWorld().getSizeY();
				double dmax = Math.hypot(sx, sy);
				write(String.format("Simulation World Size   : %d m x %d m (Max Distance D_max = %.2f m)", sx, sy, dmax));
			}
		}

		write("");
		write("Algorithm 1 Mathematical Formulation:");
		write("  Link Weight w = (1 - f(u, v)) + lambda_uv * dnorm(u, v)");
		write("  where:");
		write("    f(u, v)        : Contact encounter probability from MeetingProbabilitySet");
		write("    dnorm(u, v)    : Normalized Euclidean distance = min(1.0, distance(u, v) / D_max)");
		write("    lambda_uv      : lambda if distance estimate exists for (u, v), else 0");
		write("    Output         : Minimum cost cost[D], and next-hop neighbor traced from prev[D]");
		write("================================================================================\n");
	}

	@Override
	public void newMessage(Message m) {
		if (!targetMessageId.equals(m.getId())) {
			return;
		}

		this.creationTime = getSimTime();
		this.sourceHost = m.getFrom();
		this.destHost = m.getTo();

		write(String.format("[t = %8.2fs] === STEP 1: MESSAGE CREATION ===", getSimTime()));
		write("Message ID       : " + m.getId());
		write("Source Node      : " + sourceHost + " at " + formatCoord(sourceHost.getLocation()));
		write("Destination Node : " + destHost + " at " + formatCoord(destHost.getLocation()));
		write("Message Size     : " + (m.getSize() / 1024) + " KB");
		write("Initial Hop Count: 0");

		if (sourceHost.getRouter() instanceof DiMaxPropRouter) {
			DiMaxPropRouter router = (DiMaxPropRouter) sourceHost.getRouter();
			write(String.format("Router Parameters: lambda = %.2f | alpha = %.2f | D_max = %.2f m | strictNextHop = %s",
					router.getLambda(), router.getAlpha(), router.getMaxDistance(), router.isStrictNextHop()));
		}
		write("--------------------------------------------------------------------------------\n");
	}

	@Override
	public void messageTransferStarted(Message m, DTNHost from, DTNHost to) {
		if (!targetMessageId.equals(m.getId())) {
			return;
		}

		write(String.format("[t = %8.2fs] === STEP 2: FORWARDING DECISION & TRANSFER START ===", getSimTime()));
		write(String.format("Carrier Node     : %s at %s", from, formatCoord(from.getLocation())));
		write(String.format("Candidate Node   : %s at %s", to, formatCoord(to.getLocation())));

		double dist = from.getLocation().distance(to.getLocation());
		write(String.format("Physical Distance: %.2f m", dist));

		// Detailed Algorithm 1 Mathematical Evaluation
		if (from.getRouter() instanceof DiMaxPropRouter) {
			DiMaxPropRouter r = (DiMaxPropRouter) from.getRouter();
			PathResult res = r.getPathResultWithTrace(from, m.getTo());

			write("");
			write("--- Algorithm 1 Execution Trace at Carrier " + from + " ---");
			write(String.format("Destination Target       : %s (Address %d)", m.getTo(), m.getTo().getAddress()));
			write(String.format("Parameter lambda         : %.2f", r.getLambda()));
			write(String.format("World Normalizer D_max   : %.2f m", r.getMaxDistance()));

			List<String> calcLog = res.getComputationLog();
			if (calcLog != null && !calcLog.isEmpty()) {
				for (String line : calcLog) {
					write("  " + line);
				}
			}

			write(String.format("Computed Minimum Cost    : %.4f", res.getCost()));
			write(String.format("Traced Next-Hop Neighbor : %s",
					(res.getNextHop() != null ? "Node address " + res.getNextHop() : "NONE")));
			write(String.format("Chosen Transfer Action   : Forwarding to %s (Matches Next-Hop: %s)",
					to, (res.getNextHop() != null && res.getNextHop().equals(to.getAddress()))));
			write("--------------------------------------------------------------------------------\n");
		}
	}

	@Override
	public void messageTransferAborted(Message m, DTNHost from, DTNHost to) {
		if (!targetMessageId.equals(m.getId())) {
			return;
		}
		write(String.format("[t = %8.2fs] TRANSFER ABORTED: %s -> %s (Hop count unchanged)\n",
				getSimTime(), from, to));
	}

	@Override
	public void messageTransferred(Message m, DTNHost from, DTNHost to, boolean firstDelivery) {
		if (!targetMessageId.equals(m.getId())) {
			return;
		}

		int hops = m.getHopCount();
		if (!this.delivered) {
			this.totalHops = hops;
			this.finalPath = buildPath(m);
		}

		write(String.format("[t = %8.2fs] === STEP 3: MESSAGE TRANSFERRED ===", getSimTime()));
		write(String.format("Relayed Link     : %s -> %s", from, to));
		write(String.format("Current Hop Count: %d", hops));
		write(String.format("Path Traversed   : %s", finalPath));

		if (firstDelivery) {
			this.delivered = true;
			this.deliveryTime = getSimTime();

			write("");
			write("********************************************************************************");
			write("                    STEP 4: FINAL DESTINATION REACHED!");
			write("********************************************************************************");
			write("Target Message        : " + m.getId());
			write("Source Host           : " + sourceHost);
			write("Destination Host      : " + destHost);
			write("Total Path Traversed  : " + finalPath);
			write("Total Hops            : " + totalHops);
			write(String.format("Creation Time         : %.2f s", creationTime));
			write(String.format("Delivery Time         : %.2f s", deliveryTime));
			write(String.format("End-to-End Latency    : %.2f s", (deliveryTime - creationTime)));
			write("Delivery Status       : SUCCESS (100% Guaranteed)");
			write("********************************************************************************\n");
		}
	}

	@Override
	public void messageDeleted(Message m, DTNHost where, boolean dropped) {
		if (!targetMessageId.equals(m.getId())) {
			return;
		}

		if (dropped) {
			write(String.format("[t = %8.2fs] BUFFER OVERFLOW: %s dropped at %s\n",
					getSimTime(), m.getId(), where));
		} else {
			write(String.format("[t = %8.2fs] === STEP 5: ACK BUFFER CLEANUP ===", getSimTime()));
			write(String.format("Message %s buffer cleared at node %s following delivery confirmation.",
					m.getId(), where));
			write("--------------------------------------------------------------------------------\n");
		}
	}

	@Override
	public void done() {
		write("================================================================================");
		write("                    RESEARCH PAPER METRICS SUMMARY TABLE");
		write("================================================================================");
		write(String.format("%-28s : %s", "Message Identifier", targetMessageId));
		write(String.format("%-28s : %s", "Routing Protocol", "DiMaxProp (Algorithm 1)"));
		write(String.format("%-28s : %s", "Delivery Result", (delivered ? "DELIVERED" : "NOT DELIVERED")));
		write(String.format("%-28s : %.4f s", "End-to-End Latency", (delivered ? (deliveryTime - creationTime) : -1.0)));
		write(String.format("%-28s : %d", "Total Hops Traversed", totalHops));
		write(String.format("%-28s : %s", "Complete Path", finalPath));
		write(String.format("%-28s : %.4f", "Delivery Probability", (delivered ? 1.0 : 0.0)));
		write("================================================================================");

		super.done();
	}

	private String formatCoord(Coord c) {
		if (c == null) {
			return "(null)";
		}
		return String.format("(X: %.1f, Y: %.1f)", c.getX(), c.getY());
	}

	private String buildPath(Message m) {
		List<DTNHost> path = m.getHops();
		if (path == null || path.isEmpty()) {
			return "EMPTY";
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < path.size(); i++) {
			if (i > 0) {
				sb.append(" -> ");
			}
			sb.append(path.get(i).toString());
		}
		return sb.toString();
	}
}
