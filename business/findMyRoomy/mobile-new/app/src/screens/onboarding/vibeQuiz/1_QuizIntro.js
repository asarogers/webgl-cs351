// Quizintro.tsx / .js - Improved UI/UX Version
import { Ionicons } from "@expo/vector-icons";
import { Picker } from "@react-native-picker/picker";
import React, { useEffect, useMemo, useState } from "react";
import {
    Animated,
    Dimensions, Haptics,
    StyleSheet,
    Switch,
    Text, TouchableOpacity,
    View
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { QUIZ as QUIZ_SCHEMA } from "../../../../../components/Interests_lifestyle";

const { width } = Dimensions.get('window');

export default function Quizintro({
  selected,
  onSelect,
  mode = "minimum",
  allowOpenEnded = mode === "maximum",
}) {
  // Schema and presets logic (unchanged)
  const NODE = useMemo(
    () =>
      mode === "minimum"
        ? QUIZ_SCHEMA.sections.basics.questions.minimum_stay
        : QUIZ_SCHEMA.sections.basics.questions.maximum_stay,
    [mode]
  );

  const PRESETS = useMemo(() => {
    const order = NODE?.presetsOrder ?? [];
    const map = NODE?.presets ?? {};
    return order.map((id) => ({ id, ...map[id] })).filter(Boolean);
  }, [NODE]);

  const MIN_MONTHS = NODE?.min_value_months ?? 1;
  const MAX_MONTHS = NODE?.max_value_months ?? 240;

  // State management
  const initialMonths =
    typeof selected === "number"
      ? Math.max(MIN_MONTHS, Math.min(MAX_MONTHS, selected))
      : 3;

  const [years, setYears] = useState(Math.floor(initialMonths / 12));
  const [months, setMonths] = useState(initialMonths % 12);
  const [openEnded, setOpenEnded] = useState(selected === "open");
  const [showCustomPickers, setShowCustomPickers] = useState(false);

  const totalMonths = useMemo(() => {
    const total = years * 12 + months;
    return Math.max(MIN_MONTHS, Math.min(MAX_MONTHS, total));
  }, [years, months, MIN_MONTHS, MAX_MONTHS]);

  // Enhanced animations
  const [fadeAnim] = useState(new Animated.Value(0));
  const [slideAnim] = useState(new Animated.Value(30));
  const [scaleAnim] = useState(new Animated.Value(0.95));
  const [pickersAnim] = useState(new Animated.Value(0));

  useEffect(() => {
    Animated.parallel([
      Animated.timing(fadeAnim, { 
        toValue: 1, 
        duration: 800, 
        useNativeDriver: true 
      }),
      Animated.timing(slideAnim, { 
        toValue: 0, 
        duration: 800, 
        useNativeDriver: true 
      }),
      Animated.spring(scaleAnim, {
        toValue: 1,
        tension: 100,
        friction: 8,
        useNativeDriver: true
      })
    ]).start();
  }, [fadeAnim, slideAnim, scaleAnim]);

  // Animate custom pickers
  useEffect(() => {
    Animated.timing(pickersAnim, {
      toValue: showCustomPickers ? 1 : 0,
      duration: 300,
      useNativeDriver: false
    }).start();
  }, [showCustomPickers, pickersAnim]);

  // Emit changes to parent
  useEffect(() => {
    const next = allowOpenEnded && openEnded ? "open" : totalMonths;
    if (next !== selected) {
      onSelect?.(next);
    }
  }, [allowOpenEnded, openEnded, totalMonths, selected, onSelect]);

  const handlePreset = (m) => {
    // Add haptic feedback
    if (Haptics?.impactAsync) {
      Haptics.impactAsync(Haptics.ImpactFeedbackStyle.Light);
    }
    
    if (m == null) {
      if (allowOpenEnded) setOpenEnded(true);
      setShowCustomPickers(false);
      return;
    }
    
    const y = Math.floor(m / 12);
    const mo = m % 12;
    setYears(y);
    setMonths(mo);
    if (openEnded) setOpenEnded(false);
    setShowCustomPickers(false);
  };

  const toggleCustomPickers = () => {
    if (Haptics?.impactAsync) {
      Haptics.impactAsync(Haptics.ImpactFeedbackStyle.Medium);
    }
    setShowCustomPickers(!showCustomPickers);
    if (openEnded) setOpenEnded(false);
  };

  const handleWheelChange = (type, value) => {
    if (Haptics?.selectionAsync) {
      Haptics.selectionAsync();
    }
    
    if (type === 'years') {
      setYears(value);
    } else {
      setMonths(value);
    }
    if (openEnded) setOpenEnded(false);
  };

  const formatDuration = (totalMonths, isOpenEnded = false) => {
    if (isOpenEnded) return "Open-ended";
    
    const yrs = Math.floor(totalMonths / 12);
    const mos = totalMonths % 12;
    
    if (yrs === 0) return `${mos} month${mos !== 1 ? 's' : ''}`;
    if (mos === 0) return `${yrs} year${yrs !== 1 ? 's' : ''}`;
    return `${yrs} year${yrs !== 1 ? 's' : ''}, ${mos} month${mos !== 1 ? 's' : ''}`;
  };

  const headerTitle =
    mode === "minimum" ? "Minimum stay duration" : "Maximum stay duration";
  const headerSubtitle =
    mode === "minimum"
      ? "How long do you want to stay at minimum?"
      : allowOpenEnded 
        ? "What's the longest you'd like to stay?"
        : "What's your maximum stay duration?";

  const yearOptions = useMemo(() => {
    const arr = [];
    for (let y = 0; y <= Math.floor(MAX_MONTHS / 12); y++) arr.push(y);
    return arr;
  }, [MAX_MONTHS]);

  const monthOptions = useMemo(() => Array.from({ length: 12 }, (_, m) => m), []);

  return (
    <SafeAreaView style={styles.container}>
      <Animated.View 
        style={[
          styles.content, 
          { 
            opacity: fadeAnim, 
            transform: [
              { translateY: slideAnim },
              { scale: scaleAnim }
            ] 
          }
        ]}
      >
        <View style={styles.card}>
          {/* Enhanced Header */}
          <View style={styles.questionHeader}>
            <View style={styles.questionIconContainer}>
              <View style={styles.questionIcon}>
                <Ionicons name="time-outline" size={28} color="#3B82F6" />
              </View>
              <View style={styles.iconGlow} />
            </View>
            <Text style={styles.questionText}>{headerTitle}</Text>
            <Text style={styles.questionSubtext}>{headerSubtitle}</Text>
          </View>

          {/* Enhanced Presets with better spacing */}
          <View style={styles.presetsSection}>
            <Text style={styles.sectionTitle}>Quick options</Text>
            <View style={styles.presetsGrid}>
              {PRESETS.map((p) => {
                const isOpen = p.months == null;
                const isActive =
                  (isOpen && allowOpenEnded && openEnded) ||
                  (!isOpen && !openEnded && totalMonths === p.months);
                
                return (
                  <TouchableOpacity
                    key={p.id}
                    onPress={() => handlePreset(p.months)}
                    style={[
                      styles.presetChip,
                      isActive && styles.presetChipActive,
                    ]}
                    activeOpacity={0.8}
                  >
                    <Text style={[
                      styles.presetText, 
                      isActive && styles.presetTextActive
                    ]}>
                      {p.label}
                    </Text>
                    {isActive && (
                      <View style={styles.activeIndicator}>
                        <Ionicons name="checkmark" size={12} color="#3B82F6" />
                      </View>
                    )}
                  </TouchableOpacity>
                );
              })}
            </View>
          </View>

          {/* Custom Duration Toggle */}
          <TouchableOpacity 
            style={styles.customToggle}
            onPress={toggleCustomPickers}
            activeOpacity={0.8}
          >
            <View style={styles.customToggleLeft}>
              <Ionicons 
                name="options-outline" 
                size={18} 
                color="#6B7280" 
              />
              <Text style={styles.customToggleText}>Custom duration</Text>
            </View>
            <View style={styles.customToggleRight}>
              {!openEnded && !PRESETS.find(p => p.months === totalMonths) && (
                <View style={styles.customActiveBadge}>
                  <Text style={styles.customActiveBadgeText}>Active</Text>
                </View>
              )}
              <Ionicons 
                name={showCustomPickers ? "chevron-up" : "chevron-down"}
                size={20} 
                color="#6B7280" 
              />
            </View>
          </TouchableOpacity>

          {/* Custom Pickers with smooth animation */}
          <Animated.View style={[
            styles.customPickersContainer,
            {
              opacity: pickersAnim,
              maxHeight: pickersAnim.interpolate({
                inputRange: [0, 1],
                outputRange: [0, 200]
              }),
              marginTop: pickersAnim.interpolate({
                inputRange: [0, 1],
                outputRange: [0, 16]
              })
            }
          ]}>
            <View style={styles.wheelsWrap}>
              <View style={styles.wheelCol}>
                <Text style={styles.wheelLabel}>Years</Text>
                <View style={styles.pickerFrame}>
                  <Picker
                    selectedValue={years}
                    onValueChange={(v) => handleWheelChange('years', v)}
                    style={styles.picker}
                    itemStyle={styles.pickerItem}
                  >
                    {yearOptions.map((y) => (
                      <Picker.Item 
                        key={`y-${y}`} 
                        label={`${y}`} 
                        value={y}
                        style={styles.pickerItemText}
                      />
                    ))}
                  </Picker>
                </View>
              </View>

              <View style={styles.wheelCol}>
                <Text style={styles.wheelLabel}>Months</Text>
                <View style={styles.pickerFrame}>
                  <Picker
                    selectedValue={months}
                    onValueChange={(v) => handleWheelChange('months', v)}
                    style={styles.picker}
                    itemStyle={styles.pickerItem}
                  >
                    {monthOptions.map((m) => (
                      <Picker.Item 
                        key={`m-${m}`} 
                        label={`${m}`} 
                        value={m}
                        style={styles.pickerItemText}
                      />
                    ))}
                  </Picker>
                </View>
              </View>
            </View>
          </Animated.View>

          {/* Enhanced Open-ended toggle */}
          {allowOpenEnded && (
            <View style={styles.toggleSection}>
              <View style={styles.toggleRow}>
                <View style={styles.toggleLabelContainer}>
                  <Ionicons name="infinite-outline" size={20} color="#6366F1" />
                  <Text style={styles.toggleLabel}>No time limit</Text>
                  <Text style={styles.toggleSubtext}>Stay as long as you want</Text>
                </View>
                <Switch
                  value={openEnded}
                  onValueChange={(value) => {
                    if (Haptics?.impactAsync) {
                      Haptics.impactAsync(Haptics.ImpactFeedbackStyle.Medium);
                    }
                    setOpenEnded(value);
                  }}
                  trackColor={{ false: "#E5E7EB", true: "#C7D2FE" }}
                  thumbColor={openEnded ? "#6366F1" : "#F9FAFB"}
                  ios_backgroundColor="#E5E7EB"
                />
              </View>
            </View>
          )}

          {/* Enhanced Summary */}
          <View style={styles.summaryContainer}>
            <View style={styles.summaryPill}>
              <View style={styles.summaryIconContainer}>
                <Ionicons 
                  name={allowOpenEnded && openEnded ? "infinite" : "time"} 
                  size={18} 
                  color="#059669" 
                />
              </View>
              <Text style={styles.summaryText}>
                {mode === "minimum" ? "Minimum: " : "Maximum: "}
                {formatDuration(totalMonths, allowOpenEnded && openEnded)}
              </Text>
            </View>
          </View>
        </View>
      </Animated.View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { 
    backgroundColor: "#F8FAFC", 
    flex: 1 
  },
  content: { 
    flex: 1, 
    paddingHorizontal: 20, 
    paddingTop: 20, 
    justifyContent: "center" 
  },
  card: {
    backgroundColor: "white", 
    borderRadius: 28, 
    padding: 24,
    shadowColor: "#000", 
    shadowOpacity: 0.08, 
    shadowOffset: { width: 0, height: 12 },
    shadowRadius: 32, 
    elevation: 16,
  },

  // Enhanced Header
  questionHeader: { 
    alignItems: "center", 
    marginBottom: 32 
  },
  questionIconContainer: {
    position: "relative",
    marginBottom: 20
  },
  questionIcon: {
    width: 64, 
    height: 64, 
    borderRadius: 32, 
    backgroundColor: "#EEF2FF",
    justifyContent: "center", 
    alignItems: "center",
    borderWidth: 3,
    borderColor: "#E0E7FF",
  },
  iconGlow: {
    position: "absolute",
    width: 64,
    height: 64,
    borderRadius: 32,
    backgroundColor: "#3B82F6",
    opacity: 0.1,
    top: 0,
    left: 0,
  },
  questionText: { 
    fontSize: 26, 
    fontWeight: "800", 
    color: "#111827", 
    textAlign: "center", 
    marginBottom: 8,
    lineHeight: 32
  },
  questionSubtext: { 
    fontSize: 16, 
    color: "#6B7280", 
    textAlign: "center",
    lineHeight: 22
  },

  // Enhanced Presets
  presetsSection: {
    marginBottom: 24
  },
  sectionTitle: {
    fontSize: 16,
    fontWeight: "600",
    color: "#374151",
    marginBottom: 12
  },
  presetsGrid: { 
    flexDirection: "row", 
    flexWrap: "wrap", 
    gap: 12,
    justifyContent: "center"
  },
  presetChip: {
    paddingHorizontal: 16, 
    paddingVertical: 12, 
    borderRadius: 20,
    backgroundColor: "#F8FAFC", 
    borderColor: "#E2E8F0", 
    borderWidth: 2,
    minWidth: width * 0.25,
    alignItems: "center",
    position: "relative"
  },
  presetChipActive: {
    borderColor: "#3B82F6", 
    backgroundColor: "#EEF2FF",
    transform: [{ scale: 1.02 }]
  },
  presetText: { 
    fontSize: 14, 
    color: "#475569",
    fontWeight: "500"
  },
  presetTextActive: {
    color: "#1D4ED8",
    fontWeight: "700"
  },
  activeIndicator: {
    position: "absolute",
    top: -6,
    right: -6,
    width: 20,
    height: 20,
    borderRadius: 10,
    backgroundColor: "#EEF2FF",
    borderWidth: 2,
    borderColor: "#3B82F6",
    justifyContent: "center",
    alignItems: "center"
  },

  // Custom Toggle
  customToggle: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingVertical: 16,
    paddingHorizontal: 16,
    backgroundColor: "#F9FAFB",
    borderRadius: 16,
    borderWidth: 1,
    borderColor: "#E5E7EB",
    marginBottom: 8
  },
  customToggleLeft: {
    flexDirection: "row",
    alignItems: "center",
    gap: 10
  },
  customToggleText: {
    fontSize: 16,
    fontWeight: "600",
    color: "#374151"
  },
  customToggleRight: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8
  },
  customActiveBadge: {
    backgroundColor: "#10B981",
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 12
  },
  customActiveBadgeText: {
    color: "white",
    fontSize: 11,
    fontWeight: "600"
  },

  // Custom Pickers
  customPickersContainer: {
    overflow: "hidden"
  },
  wheelsWrap: { 
    flexDirection: "row", 
    gap: 16 
  },
  wheelCol: { 
    flex: 1 
  },
  wheelLabel: { 
    fontSize: 14, 
    color: "#6B7280", 
    marginBottom: 8, 
    textAlign: "center",
    fontWeight: "600"
  },
  pickerFrame: { 
    borderWidth: 2, 
    borderColor: "#E2E8F0", 
    borderRadius: 20, 
    backgroundColor: "#FAFBFC", 
    overflow: "hidden" 
  },
  picker: { 
    height: 140 
  },
  pickerItem: {
    fontSize: 16,
    color: "#374151"
  },
  pickerItemText: {
    fontSize: 16,
    color: "#374151"
  },

  // Enhanced Toggle
  toggleSection: {
    marginTop: 20,
    marginBottom: 8
  },
  toggleRow: { 
    paddingHorizontal: 16,
    paddingVertical: 16,
    flexDirection: "row", 
    alignItems: "center", 
    justifyContent: "space-between",
    backgroundColor: "#FEFBFF",
    borderRadius: 16,
    borderWidth: 2,
    borderColor: "#E0E7FF"
  },
  toggleLabelContainer: {
    flexDirection: "column",
    gap: 2
  },
  toggleLabel: { 
    fontSize: 16, 
    color: "#111827",
    fontWeight: "600",
    flexDirection: "row",
    alignItems: "center"
  },
  toggleSubtext: {
    fontSize: 13,
    color: "#6B7280"
  },

  // Enhanced Summary
  summaryContainer: {
    marginTop: 24,
    alignItems: "center"
  },
  summaryPill: {
    flexDirection: "row", 
    alignItems: "center",
    gap: 12,
    backgroundColor: "#ECFDF5", 
    borderWidth: 2, 
    borderColor: "#A7F3D0",
    paddingHorizontal: 20, 
    paddingVertical: 14, 
    borderRadius: 24,
    minWidth: width * 0.6,
    justifyContent: "center"
  },
  summaryIconContainer: {
    width: 24,
    height: 24,
    borderRadius: 12,
    backgroundColor: "#D1FAE5",
    justifyContent: "center",
    alignItems: "center"
  },
  summaryText: { 
    color: "#065F46", 
    fontSize: 15, 
    fontWeight: "700",
    textAlign: "center"
  },
});