import React, { useState, useEffect, useRef } from "react";
import {
  View,
  Text,
  StyleSheet,
  TouchableOpacity,
  Dimensions,
  Animated,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { Ionicons } from "@expo/vector-icons";
import Svg, { Circle } from "react-native-svg";

const { width } = Dimensions.get("window");

export const ZoneDrawingAnimation = ({ onNext }) => {
  const [isPlaying, setIsPlaying] = useState(true);
  const [animationProgress] = useState(new Animated.Value(0));
  const [currentProgress, setCurrentProgress] = useState(0);

  const MAP_WIDTH = 280;
  const MAP_HEIGHT = 200;
  const ANIMATION_DURATION = 3500;
  const CENTER_X = MAP_WIDTH / 2;
  const CENTER_Y = MAP_HEIGHT / 2;

  // Animation phases:
  // 0.0 - 0.2: Fingers appear and move to initial position
  // 0.2 - 0.7: Circle expands (fingers move apart)
  // 0.7 - 0.85: Brief pause at max size
  // 0.85 - 1.0: Circle shrinks (fingers move together)

  const getFingerPositions = (progress) => {
    let finger1, finger2, circleRadius;

    if (progress <= 0.2) {
      // Fingers appear and move to starting position
      const t = progress / 0.2;
      const startRadius = 20;
      finger1 = {
        x: CENTER_X - startRadius * t,
        y: CENTER_Y,
        opacity: t
      };
      finger2 = {
        x: CENTER_X + startRadius * t,
        y: CENTER_Y,
        opacity: t
      };
      circleRadius = startRadius * t;
    } else if (progress <= 0.7) {
      // Expanding phase
      const t = (progress - 0.2) / 0.5;
      const easedT = 1 - Math.pow(1 - t, 3); // Ease out cubic
      const minRadius = 20;
      const maxRadius = 80;
      circleRadius = minRadius + (maxRadius - minRadius) * easedT;
      
      finger1 = {
        x: CENTER_X - circleRadius,
        y: CENTER_Y,
        opacity: 1
      };
      finger2 = {
        x: CENTER_X + circleRadius,
        y: CENTER_Y,
        opacity: 1
      };
    } else if (progress <= 0.85) {
      // Brief pause at max size
      circleRadius = 80;
      finger1 = {
        x: CENTER_X - circleRadius,
        y: CENTER_Y,
        opacity: 1
      };
      finger2 = {
        x: CENTER_X + circleRadius,
        y: CENTER_Y,
        opacity: 1
      };
    } else {
      // Shrinking phase
      const t = (progress - 0.85) / 0.15;
      const easedT = Math.pow(t, 2); // Ease in quad
      const maxRadius = 80;
      const minRadius = 20;
      circleRadius = maxRadius - (maxRadius - minRadius) * easedT;
      
      finger1 = {
        x: CENTER_X - circleRadius,
        y: CENTER_Y,
        opacity: 1 - (t * 0.3) // Slight fade
      };
      finger2 = {
        x: CENTER_X + circleRadius,
        y: CENTER_Y,
        opacity: 1 - (t * 0.3)
      };
    }

    return { finger1, finger2, circleRadius };
  };

  useEffect(() => {
    const startAnimation = () => {
      animationProgress.setValue(0);
      Animated.timing(animationProgress, {
        toValue: 1,
        duration: ANIMATION_DURATION,
        useNativeDriver: false,
      }).start(() => {
        if (isPlaying) {
          setTimeout(startAnimation, 800); // Pause before restart
        }
      });
    };

    const listener = animationProgress.addListener(({ value }) => {
      setCurrentProgress(value);
    });

    if (isPlaying) {
      startAnimation();
    }

    return () => {
      animationProgress.removeListener(listener);
    };
  }, [isPlaying]);

  const toggleAnimation = () => {
    setIsPlaying(!isPlaying);
  };

  const { finger1, finger2, circleRadius } = getFingerPositions(currentProgress);
  const circleOpacity = Math.min(1, currentProgress * 3);

  return (
    <SafeAreaView style={styles.safeArea}>
      <View style={styles.card}>
        {/* Header */}
        <View style={styles.questionHeader}>
          <View style={styles.questionIcon}>
            <Ionicons name="radio-button-on" size={24} color="#3B82F6" />
          </View>
          <Text style={styles.questionText}>
            Draw Your Perfect Zone
          </Text>
          <Text style={styles.questionSubtext}>
            Find roommates where you actually want to live
          </Text>
        </View>

        {/* Animation Container */}
        <View style={styles.animationContainer}>
          <View style={styles.mapContainer}>
            {/* Background */}
            <View style={styles.mapBackground} />
            
            {/* Mock map points */}
            <View style={[styles.mapPoint, styles.greenPoint, { top: 16, left: 24 }]} />
            <View style={[styles.mapPoint, styles.redPoint, { top: 48, right: 32 }]} />
            <View style={[styles.mapPoint, styles.yellowPoint, { bottom: 32, left: 48 }]} />
            <View style={[styles.mapPoint, styles.purplePoint, { bottom: 24, right: 24 }]} />

            {/* Zone Label */}
            <View style={styles.zoneLabel}>
              <Text style={styles.zoneLabelText}>My Zone</Text>
            </View>

            {/* SVG Circle */}
            <Svg style={styles.svgOverlay} viewBox={`0 0 ${MAP_WIDTH} ${MAP_HEIGHT}`}>
              {/* Zone circle fill */}
              <Circle
                cx={CENTER_X}
                cy={CENTER_Y}
                r={circleRadius}
                fill="rgba(59, 130, 246, 0.15)"
                opacity={circleOpacity}
              />
              
              {/* Zone circle stroke */}
              <Circle
                cx={CENTER_X}
                cy={CENTER_Y}
                r={circleRadius}
                fill="none"
                stroke="#3B82F6"
                strokeWidth="3"
                opacity={circleOpacity}
              />
            </Svg>

            {/* Finger 1 */}
            {currentProgress > 0.05 && (
              <Animated.View
                style={[
                  styles.fingerIndicator,
                  styles.finger1,
                  {
                    left: finger1.x - 16,
                    top: finger1.y - 16,
                    opacity: finger1.opacity
                  }
                ]}
              >
                <View style={styles.fingerPing} />
                <View style={styles.fingerCore} />
              </Animated.View>
            )}
            
            {/* Finger 2 */}
            {currentProgress > 0.05 && (
              <Animated.View
                style={[
                  styles.fingerIndicator,
                  styles.finger2,
                  {
                    left: finger2.x - 16,
                    top: finger2.y - 16,
                    opacity: finger2.opacity
                  }
                ]}
              >
                <View style={[styles.fingerPing, styles.fingerPing2]} />
                <View style={[styles.fingerCore, styles.fingerCore2]} />
              </Animated.View>
            )}

            {/* Connection line between fingers (subtle) */}
            {currentProgress > 0.2 && (
              <View
                style={[
                  styles.connectionLine,
                  {
                    left: finger1.x,
                    top: finger1.y - 1,
                    width: finger2.x - finger1.x,
                    opacity: 0.3 * finger1.opacity
                  }
                ]}
              />
            )}
          </View>
        </View>

        {/* Play/Pause Button */}
        <View style={styles.buttonContainer}>
          <TouchableOpacity
            style={styles.playButton}
            onPress={toggleAnimation}
            activeOpacity={0.8}
          >
            <Ionicons
              name={isPlaying ? "pause" : "play"}
              size={20}
              color="white"
            />
            <Text style={styles.playButtonText}>
              {isPlaying ? "Pause" : "Play"}
            </Text>
          </TouchableOpacity>
        </View>

        {/* Instructions */}
        <View style={styles.instructionsContainer}>
          <Text style={styles.instructionsText}>
            Pinch to expand or shrink your search zone. Set your budget, safety preferences, and pet policies for each area.
          </Text>
        </View>
      </View>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: "#F8FAFC",
  },
  card: {
    marginTop: 0,
    backgroundColor: "white",
    borderRadius: 24,
    padding: 20,
    shadowColor: "#000",
    shadowOpacity: 0.1,
    shadowOffset: { width: 0, height: 8 },
    shadowRadius: 24,
    elevation: 12,
    marginHorizontal: 0,
    flex: 1,
    justifyContent: "center",
  },
  questionHeader: {
    alignItems: "center",
    marginBottom: 32,
  },
  questionIcon: {
    width: 56,
    height: 56,
    borderRadius: 28,
    backgroundColor: "#EEF2FF",
    justifyContent: "center",
    alignItems: "center",
    marginBottom: 20,
  },
  questionText: {
    fontSize: 24,
    fontWeight: "800",
    color: "#111827",
    textAlign: "center",
    marginBottom: 12,
    lineHeight: 32,
  },
  questionSubtext: {
    fontSize: 16,
    color: "#64748B",
    textAlign: "center",
    lineHeight: 20,
  },
  animationContainer: {
    backgroundColor: "#F8FAFC",
    borderRadius: 20,
    padding: 24,
    marginBottom: 24,
    alignItems: "center",
  },
  mapContainer: {
    width: 280,
    height: 200,
    position: "relative",
  },
  mapBackground: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: "#DBEAFE",
    borderRadius: 12,
  },
  mapPoint: {
    position: "absolute",
    width: 8,
    height: 8,
    borderRadius: 4,
  },
  greenPoint: {
    backgroundColor: "#10B981",
  },
  redPoint: {
    backgroundColor: "#EF4444",
  },
  yellowPoint: {
    backgroundColor: "#F59E0B",
  },
  purplePoint: {
    backgroundColor: "#8B5CF6",
  },
  zoneLabel: {
    position: "absolute",
    top: "50%",
    left: "50%",
    transform: [{ translateX: -35 }, { translateY: -15 }],
    backgroundColor: "#3B82F6",
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderRadius: 8,
    shadowColor: "#000",
    shadowOpacity: 0.25,
    shadowOffset: { width: 0, height: 2 },
    shadowRadius: 4,
    elevation: 4,
  },
  zoneLabelText: {
    color: "white",
    fontSize: 14,
    fontWeight: "600",
  },
  svgOverlay: {
    position: "absolute",
    top: 0,
    left: 0,
    width: 280,
    height: 200,
  },
  fingerIndicator: {
    position: "absolute",
    width: 32,
    height: 32,
    borderRadius: 16,
    backgroundColor: "white",
    borderWidth: 3,
    justifyContent: "center",
    alignItems: "center",
    shadowColor: "#000",
    shadowOpacity: 0.25,
    shadowOffset: { width: 0, height: 2 },
    shadowRadius: 4,
    elevation: 4,
  },
  finger1: {
    borderColor: "#3B82F6",
  },
  finger2: {
    borderColor: "#60A5FA",
  },
  fingerPing: {
    position: "absolute",
    width: 32,
    height: 32,
    borderRadius: 16,
    backgroundColor: "#3B82F6",
    opacity: 0.2,
  },
  fingerPing2: {
    backgroundColor: "#60A5FA",
  },
  fingerCore: {
    width: 8,
    height: 8,
    borderRadius: 4,
    backgroundColor: "#3B82F6",
  },
  fingerCore2: {
    backgroundColor: "#60A5FA",
  },
  connectionLine: {
    position: "absolute",
    height: 2,
    backgroundColor: "#3B82F6",
    borderRadius: 1,
  },
  buttonContainer: {
    alignItems: "center",
    marginBottom: 24,
  },
  playButton: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#3B82F6",
    paddingHorizontal: 24,
    paddingVertical: 12,
    borderRadius: 25,
    shadowColor: "#000",
    shadowOpacity: 0.25,
    shadowOffset: { width: 0, height: 2 },
    shadowRadius: 4,
    elevation: 4,
  },
  playButtonText: {
    color: "white",
    fontSize: 16,
    fontWeight: "600",
    marginLeft: 8,
  },
  instructionsContainer: {
    paddingHorizontal: 8,
  },
  instructionsText: {
    fontSize: 14,
    color: "#6B7280",
    textAlign: "center",
    lineHeight: 20,
  },
});