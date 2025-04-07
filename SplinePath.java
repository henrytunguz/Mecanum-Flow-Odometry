package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

/**
 * SplinePath - A comprehensive spline-based path generator and follower for FTC robots
 * 
 * This class allows you to:
 * 1. Create a path by adding waypoints (x, y, yaw)
 * 2. Generate a smooth spline path through those waypoints
 * 3. Sample the path at any point to get position, heading, and curvature
 */
public class SplinePath {
    
    /**
     * Represents a waypoint with position and orientation
     */
    public static class Waypoint {
        public final double x;
        public final double y;
        public final double yaw; // in radians
        
        public Waypoint(double x, double y, double yaw) {
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }
    }
    
    /**
     * Represents a point along the path with position, heading, and curvature
     */
    public static class PathPoint {
        public final double x;
        public final double y;
        public final double heading; // in radians
        public final double curvature;
        public final double distanceAlongPath;
        
        public PathPoint(double x, double y, double heading, double curvature, double distanceAlongPath) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.curvature = curvature;
            this.distanceAlongPath = distanceAlongPath;
        }
    }
    
    private List<Waypoint> waypoints = new ArrayList<>();
    private List<PathPoint> generatedPath = new ArrayList<>();
    private double pathResolution = 0.01; // meters between path points
    private boolean pathGenerated = false;
    
    /**
     * Add a waypoint to the path
     * 
     * @param x X coordinate in meters
     * @param y Y coordinate in meters
     * @param yawDegrees Heading in degrees (0 = forward, 90 = left, etc.)
     * @return this SplinePath object for method chaining
     */
    public SplinePath addWaypoint(double x, double y, double yawDegrees) {
        waypoints.add(new Waypoint(x, y, Math.toRadians(yawDegrees)));
        pathGenerated = false;
        return this;
    }
    
    /**
     * Clear all waypoints
     * 
     * @return this SplinePath object for method chaining
     */
    public SplinePath clearWaypoints() {
        waypoints.clear();
        generatedPath.clear();
        pathGenerated = false;
        return this;
    }
    
    /**
     * Set the resolution of the generated path
     * 
     * @param resolution distance in meters between generated path points
     * @return this SplinePath object for method chaining
     */
    public SplinePath setPathResolution(double resolution) {
        this.pathResolution = resolution;
        pathGenerated = false;
        return this;
    }
    
    /**
     * Generate a smooth path through all waypoints
     * 
     * @return this SplinePath object for method chaining
     */
    public SplinePath generatePath() {
        if (waypoints.size() < 2) {
            throw new IllegalStateException("Need at least 2 waypoints to generate a path");
        }
        
        generatedPath.clear();
        
        // For each pair of waypoints, generate a cubic spline segment
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Waypoint start = waypoints.get(i);
            Waypoint end = waypoints.get(i + 1);
            
            // Generate spline segment between these waypoints
            List<PathPoint> segment = generateSplineSegment(start, end);
            generatedPath.addAll(segment);
        }
        
        pathGenerated = true;
        return this;
    }
    
    /**
     * Generate a cubic spline segment between two waypoints
     */
    private List<PathPoint> generateSplineSegment(Waypoint start, Waypoint end) {
        List<PathPoint> segment = new ArrayList<>();
        
        // Calculate the distance between waypoints
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double distance = Math.sqrt(dx * dx + dy * dy);
        
        // Calculate control points for the cubic spline
        // These control points ensure the spline starts and ends with the correct heading
        double startTangentMagnitude = distance / 3.0;
        double endTangentMagnitude = distance / 3.0;
        
        double startTangentX = start.x + startTangentMagnitude * Math.cos(start.yaw);
        double startTangentY = start.y + startTangentMagnitude * Math.sin(start.yaw);
        
        double endTangentX = end.x - endTangentMagnitude * Math.cos(end.yaw);
        double endTangentY = end.y - endTangentMagnitude * Math.sin(end.yaw);
        
        // Number of points to generate along this segment
        int numPoints = Math.max(2, (int)(distance / pathResolution));
        double totalDistanceAlongPath = generatedPath.isEmpty() ? 0 : 
                                        generatedPath.get(generatedPath.size() - 1).distanceAlongPath;
        double lastX = start.x;
        double lastY = start.y;
        
        for (int i = 0; i <= numPoints; i++) {
            double t = (double) i / numPoints;
            
            // Cubic Bezier formula
            double x = cubicBezier(start.x, startTangentX, endTangentX, end.x, t);
            double y = cubicBezier(start.y, startTangentY, endTangentY, end.y, t);
            
            // Calculate derivatives for heading and curvature
            double dx_dt = cubicBezierDerivative(start.x, startTangentX, endTangentX, end.x, t);
            double dy_dt = cubicBezierDerivative(start.y, startTangentY, endTangentY, end.y, t);
            
            double d2x_dt2 = cubicBezierSecondDerivative(start.x, startTangentX, endTangentX, end.x, t);
            double d2y_dt2 = cubicBezierSecondDerivative(start.y, startTangentY, endTangentY, end.y, t);
            
            // Calculate heading (tangent angle)
            double heading = Math.atan2(dy_dt, dx_dt);
            
            // Calculate curvature
            double curvature = (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / 
                               Math.pow(dx_dt * dx_dt + dy_dt * dy_dt, 1.5);
            
            // Calculate distance increment
            if (i > 0) {
                double segmentDx = x - lastX;
                double segmentDy = y - lastY;
                totalDistanceAlongPath += Math.sqrt(segmentDx * segmentDx + segmentDy * segmentDy);
            }
            
            segment.add(new PathPoint(x, y, heading, curvature, totalDistanceAlongPath));
            
            lastX = x;
            lastY = y;
        }
        
        return segment;
    }
    
    /**
     * Cubic Bezier curve formula
     */
    private double cubicBezier(double p0, double p1, double p2, double p3, double t) {
        double oneMinusT = 1 - t;
        return Math.pow(oneMinusT, 3) * p0 + 
               3 * Math.pow(oneMinusT, 2) * t * p1 + 
               3 * oneMinusT * Math.pow(t, 2) * p2 + 
               Math.pow(t, 3) * p3;
    }
    
    /**
     * First derivative of cubic Bezier curve
     */
    private double cubicBezierDerivative(double p0, double p1, double p2, double p3, double t) {
        double oneMinusT = 1 - t;
        return 3 * Math.pow(oneMinusT, 2) * (p1 - p0) + 
               6 * oneMinusT * t * (p2 - p1) + 
               3 * Math.pow(t, 2) * (p3 - p2);
    }
    
    /**
     * Second derivative of cubic Bezier curve
     */
    private double cubicBezierSecondDerivative(double p0, double p1, double p2, double p3, double t) {
        return 6 * (1 - t) * (p2 - 2 * p1 + p0) + 
               6 * t * (p3 - 2 * p2 + p1);
    }
    
    /**
     * Get the generated path
     * 
     * @return List of path points
     */
    public List<PathPoint> getPath() {
        if (!pathGenerated) {
            generatePath();
        }
        return generatedPath;
    }
    
    /**
     * Get the total path length in meters
     * 
     * @return Path length
     */
    public double getPathLength() {
        if (!pathGenerated) {
            generatePath();
        }
        return generatedPath.get(generatedPath.size() - 1).distanceAlongPath;
    }
    
    /**
     * Get a point on the path at a specific distance along the path
     * 
     * @param distance Distance along the path in meters
     * @return The path point at that distance
     */
    public PathPoint getPointAtDistance(double distance) {
        if (!pathGenerated) {
            generatePath();
        }
        
        if (generatedPath.isEmpty()) {
            throw new IllegalStateException("Path is empty");
        }
        
        // Clamp distance to path length
        distance = Math.max(0, Math.min(distance, getPathLength()));
        
        // Binary search to find the point
        int low = 0;
        int high = generatedPath.size() - 1;
        
        while (low <= high) {
            int mid = (low + high) / 2;
            double midDistance = generatedPath.get(mid).distanceAlongPath;
            
            if (Math.abs(midDistance - distance) < 1e-6) {
                return generatedPath.get(mid);
            } else if (midDistance < distance) {
                low = mid + 1;
            } else {
                high = mid - 1;
            }
        }
        
        // Interpolate between the two closest points
        int index = Math.min(low, generatedPath.size() - 1);
        if (index > 0 && generatedPath.get(index).distanceAlongPath > distance) {
            index--;
        }
        
        PathPoint p1 = generatedPath.get(index);
        
        if (index == generatedPath.size() - 1) {
            return p1;
        }
        
        PathPoint p2 = generatedPath.get(index + 1);
        
        double t = (distance - p1.distanceAlongPath) / (p2.distanceAlongPath - p1.distanceAlongPath);
        
        double x = p1.x + t * (p2.x - p1.x);
        double y = p1.y + t * (p2.y - p1.y);
        double heading = p1.heading + t * (p2.heading - p1.heading);
        double curvature = p1.curvature + t * (p2.curvature - p1.curvature);
        
        return new PathPoint(x, y, heading, curvature, distance);
    }
    
    /**
     * Calculate the closest point on the path to a given position
     * 
     * @param robotX Robot's current X position
     * @param robotY Robot's current Y position
     * @return The closest path point
     */
    public PathPoint getClosestPoint(double robotX, double robotY) {
        if (!pathGenerated) {
            generatePath();
        }
        
        if (generatedPath.isEmpty()) {
            throw new IllegalStateException("Path is empty");
        }
        
        PathPoint closest = null;
        double minDistance = Double.MAX_VALUE;
        
        for (PathPoint point : generatedPath) {
            double dx = point.x - robotX;
            double dy = point.y - robotY;
            double distance = Math.sqrt(dx * dx + dy * dy);
            
            if (distance < minDistance) {
                minDistance = distance;
                closest = point;
            }
        }
        
        return closest;
    }
    
    /**
     * Calculate the lookahead point on the path
     * 
     * @param robotX Robot's current X position
     * @param robotY Robot's current Y position
     * @param lookaheadDistance Distance to look ahead on the path
     * @return The lookahead path point
     */
    public PathPoint getLookaheadPoint(double robotX, double robotY, double lookaheadDistance) {
        if (!pathGenerated) {
            generatePath();
        }
        
        PathPoint closest = getClosestPoint(robotX, robotY);
        double lookaheadDist = closest.distanceAlongPath + lookaheadDistance;
        
        // Clamp to path length
        lookaheadDist = Math.min(lookaheadDist, getPathLength());
        
        return getPointAtDistance(lookaheadDist);
    }
    
    /**
     * Calculate mecanum drive wheel powers to follow the path
     * 
     * @param robotX Current robot X position
     * @param robotY Current robot Y position
     * @param robotHeading Current robot heading in radians
     * @param lookaheadDistance Distance to look ahead on the path
     * @return Array of wheel powers [frontLeft, frontRight, backLeft, backRight]
     */
    public double[] getMecanumWheelPowers(double robotX, double robotY, double robotHeading, double lookaheadDistance) {
        if (!pathGenerated) {
            generatePath();
        }
        
        // Get lookahead point
        PathPoint target = getLookaheadPoint(robotX, robotY, lookaheadDistance);
        
        // Calculate vector to target point
        double dx = target.x - robotX;
        double dy = target.y - robotY;
        
        // Rotate the vector to robot's coordinate frame
        double robotCos = Math.cos(-robotHeading);
        double robotSin = Math.sin(-robotHeading);
        
        double xLocal = dx * robotCos - dy * robotSin;
        double yLocal = dx * robotSin + dy * robotCos;
        
        // Calculate heading error
        double headingError = normalizeAngle(target.heading - robotHeading);
        
        // Calculate drive components
        double xPower = 0.8 * xLocal / lookaheadDistance;
        double yPower = 0.8 * yLocal / lookaheadDistance;
        double rotationPower = 0.5 * headingError;
        
        // Normalize powers if they exceed 1.0
        double maxPower = Math.max(1.0, Math.sqrt(xPower*xPower + yPower*yPower) + Math.abs(rotationPower));
        if (maxPower > 1.0) {
            xPower /= maxPower;
            yPower /= maxPower;
            rotationPower /= maxPower;
        }
        
        // Calculate mecanum wheel powers
        double frontLeft = yPower + xPower + rotationPower;
        double frontRight = yPower - xPower - rotationPower;
        double backLeft = yPower - xPower + rotationPower;
        double backRight = yPower + xPower - rotationPower;
        
        return new double[] {frontLeft, frontRight, backLeft, backRight};
    }
    
    /**
     * Normalize an angle to the range [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    /**
     * Get motion data for a specific point along the path
     * 
     * @param distance Distance along the path
     * @return Array containing [x, y, heading in radians, curvature]
     */
    public double[] getMotionData(double distance) {
        PathPoint point = getPointAtDistance(distance);
        return new double[] {
            point.x,
            point.y,
            point.heading,
            point.curvature
        };
    }
    
    /**
     * Get the waypoints that define this path
     * 
     * @return List of waypoints
     */
    public List<Waypoint> getWaypoints() {
        return new ArrayList<>(waypoints);
    }
    
    /**
     * Create a path from a series of x, y, yaw values
     * 
     * @param xPoints Array of x coordinates
     * @param yPoints Array of y coordinates
     * @param yawDegrees Array of headings in degrees
     * @return A new SplinePath object
     */
    public static SplinePath fromXYYawArrays(double[] xPoints, double[] yPoints, double[] yawDegrees) {
        if (xPoints.length != yPoints.length || xPoints.length != yawDegrees.length) {
            throw new IllegalArgumentException("Input arrays must have the same length");
        }
        
        SplinePath path = new SplinePath();
        for (int i = 0; i < xPoints.length; i++) {
            path.addWaypoint(xPoints[i], yPoints[i], yawDegrees[i]);
        }
        
        return path;
    }
    
    /**
     * Create a path from a 2D array where each row is [x, y, yawDegrees]
     * 
     * @param waypoints 2D array of waypoints
     * @return A new SplinePath object
     */
    public static SplinePath fromWaypointArray(double[][] waypoints) {
        SplinePath path = new SplinePath();
        for (double[] point : waypoints) {
            if (point.length >= 3) {
                path.addWaypoint(point[0], point[1], point[2]);
            }
        }
        return path;
    }
    
    /**
     * Calculate the time it would take to traverse the path with given velocity constraints
     * 
     * @param maxVelocity Maximum velocity in meters per second
     * @param maxAcceleration Maximum acceleration in meters per second squared
     * @return Estimated time in seconds
     */
    public double calculateTraversalTime(double maxVelocity, double maxAcceleration) {
        if (!pathGenerated) {
            generatePath();
        }
        
        double pathLength = getPathLength();
        
        // Time to accelerate to max velocity
        double timeToMaxVel = maxVelocity / maxAcceleration;
        
        // Distance covered during acceleration
        double accelDistance = 0.5 * maxAcceleration * timeToMaxVel * timeToMaxVel;
        
        // Check if we can reach max velocity
        if (2 * accelDistance < pathLength) {
            // We can reach max velocity, so we'll have a cruise phase
            double cruiseDistance = pathLength - 2 * accelDistance;
            double cruiseTime = cruiseDistance / maxVelocity;
            return 2 * timeToMaxVel + cruiseTime;
        } else {
            // We can't reach max velocity, so we'll have a triangular profile
            return 2 * Math.sqrt(pathLength / maxAcceleration);
        }
    }
    
    /**
     * Get a simplified version of the path with fewer points
     * 
     * @param maxError Maximum allowed error in meters
     * @return A new list of path points
     */
    public List<PathPoint> getSimplifiedPath(double maxError) {
        if (!pathGenerated) {
            generatePath();
        }
        
        if (generatedPath.size() <= 2) {
            return new ArrayList<>(generatedPath);
        }
        
        List<PathPoint> simplified = new ArrayList<>();
        simplified.add(generatedPath.get(0));
        
        int i = 0;
        while (i < generatedPath.size() - 1) {
            int j = i + 2;
            while (j < generatedPath.size()) {
                // Check if points between i and j can be approximated by a straight line
                boolean canSimplify = true;
                for (int k = i + 1; k < j; k++) {
                    double error = pointToLineDistance(
                        generatedPath.get(i).x, generatedPath.get(i).y,
                        generatedPath.get(j).x, generatedPath.get(j).y,
                        generatedPath.get(k).x, generatedPath.get(k).y
                    );
                    
                    if (error > maxError) {
                        canSimplify = false;
                        break;
                    }
                }
                
                if (!canSimplify) {
                    j--;
                    break;
                }
                
                j++;
            }
            
            simplified.add(generatedPath.get(j - 1));
            i = j - 1;
        }
        
        return simplified;
    }
    
    /**
     * Calculate the distance from a point to a line defined by two points
     */
    private double pointToLineDistance(double x1, double y1, double x2, double y2, double px, double py) {
        double A = px - x1;
        double B = py - y1;
        double C = x2 - x1;
        double D = y2 - y1;
        
        double dot = A * C + B * D;
        double lenSq = C * C + D * D;
        double param = dot / lenSq;
        
        double xx, yy;
        
        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }
        
        double dx = px - xx;
        double dy = py - yy;
        
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * Visualize the path as a string (for debugging)
     * 
     * @return A string representation of the path
     */
    @Override
    public String toString() {
        if (!pathGenerated) {
            generatePath();
        }
        
        StringBuilder sb = new StringBuilder();
        sb.append("SplinePath with ").append(waypoints.size()).append(" waypoints and ")
          .append(generatedPath.size()).append(" path points\n");
        
        sb.append("Waypoints:\n");
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint wp = waypoints.get(i);
            sb.append(String.format("  %d: (%.2f, %.2f, %.1f°)\n", 
                i, wp.x, wp.y, Math.toDegrees(wp.yaw)));
        }
        
        sb.append("Path length: ").append(String.format("%.2f", getPathLength())).append(" meters\n");
        
        return sb.toString();
    }
}

