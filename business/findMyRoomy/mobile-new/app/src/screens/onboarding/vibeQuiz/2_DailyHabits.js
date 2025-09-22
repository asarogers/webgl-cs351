import { Ionicons } from "@expo/vector-icons";
import React, { useEffect, useState } from "react";
import {
    Animated,
    Dimensions,
    StyleSheet,
    Text,
    TouchableOpacity,
    View
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { QUIZ as quiz_options } from "../../../../../components/Interests_lifestyle"; // ⬅️ import the keyed schema

const { width } = Dimensions.get("window");

// --- pull the correct section/question ---
const sleepQ = quiz_options.sections.lifestyle.questions.sleep_rhythm;

// optional UI metadata for icons/colors/descriptions
/** @type {{ [key: string]: { icon: string; color: string; description: string } }} */
const SLEEP_META = {
  early: { icon: "🌅", color: "#10B981", description: "Wake up early, wind down earlier" },
  night: { icon: "🌙", color: "#F59E0B", description: "Stay up late, active at night" },
  flex:  { icon: "⏰", color: "#8B5CF6", description: "Schedule varies / adaptable" },
};

// build options array your component expects
const options = sleepQ.optionsOrder.map((id) => ({
  id,
  label: sleepQ.options[id].label,
  icon: SLEEP_META[id]?.icon ?? "🕒",
  color: SLEEP_META[id]?.color ?? "#3B82F6",
  description: SLEEP_META[id]?.description ?? "",
}));

export default function DailyHabits({ selected, onSelect, onNext }) {
  const [buttonAnimations] = useState(options.map(() => new Animated.Value(0)));

  useEffect(() => {
    const animations = buttonAnimations.map((value, index) =>
      Animated.timing(value, { toValue: 1, duration: 400, delay: index * 100, useNativeDriver: true })
    );
    Animated.stagger(100, animations).start();
  }, []);

  const handleSelect = (id) => {
    onSelect?.(id);
    const i = options.findIndex((opt) => opt.id === id);
    if (i !== -1) {
      Animated.sequence([
        Animated.timing(buttonAnimations[i], { toValue: 0.95, duration: 100, useNativeDriver: true }),
        Animated.timing(buttonAnimations[i], { toValue: 1, duration: 100, useNativeDriver: true }),
      ]).start();
    }
  };

  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.card}>
        <View style={styles.questionHeader}>
          <View style={styles.questionIcon}>
            <Ionicons name="moon" size={24} color="#3B82F6" />
          </View>
          <Text style={styles.questionText}>What's your sleep schedule?</Text>
          <Text style={styles.questionSubtext}>
            We'll match you with roommates who share similar rhythms
          </Text>
        </View>

        <View style={styles.optionsContainer}>
          {options.map((opt, idx) => (
            <Animated.View
              key={opt.id}
              style={{
                opacity: buttonAnimations[idx],
                transform: [
                  { translateY: buttonAnimations[idx].interpolate({ inputRange: [0, 1], outputRange: [20, 0] }) },
                  { scale: buttonAnimations[idx] },
                ],
              }}
            >
              <TouchableOpacity
                onPress={() => handleSelect(opt.id)}
                activeOpacity={0.8}
                style={[
                  styles.optionButton,
                  selected === opt.id && [
                    styles.optionButtonSelected,
                    { borderColor: opt.color, backgroundColor: `${opt.color}10` },
                  ],
                ]}
              >
                <View style={styles.optionContent}>
                  <View
                    style={[
                      styles.emojiContainer,
                      { backgroundColor: `${opt.color}20` },
                      selected === opt.id && { backgroundColor: `${opt.color}30`, transform: [{ scale: 1.1 }] },
                    ]}
                  >
                    <Text style={styles.emoji}>{opt.icon}</Text>
                  </View>
                  <View style={styles.optionTextContainer}>
                    <Text style={[styles.optionLabel, selected === opt.id && { color: opt.color, fontWeight: "700" }]}>
                      {opt.label}
                    </Text>
                    {!!opt.description && (
                      <Text style={[styles.optionDescription, selected === opt.id && { color: "#374151" }]}>
                        {opt.description}
                      </Text>
                    )}
                  </View>
                  {selected === opt.id && (
                    <Animated.View style={[styles.checkContainer, { backgroundColor: opt.color }]}>
                      <Ionicons name="checkmark" size={16} color="white" />
                    </Animated.View>
                  )}
                </View>
              </TouchableOpacity>
            </Animated.View>
          ))}
        </View>
      </View>
    </SafeAreaView>
  );
}

// (styles unchanged)
const styles = StyleSheet.create({
  container: { backgroundColor: "#F8FAFC", flex: 1 },
  card: {
    marginTop: 0, backgroundColor: "white", borderRadius: 24, padding: 10,
    shadowColor: "#000", shadowOpacity: 0.1, shadowOffset: { width: 0, height: 8 },
    shadowRadius: 24, elevation: 12, marginHorizontal: 0, flex: 1, justifyContent: "center",
  },
  questionHeader: { alignItems: "center", marginBottom: 32 },
  questionIcon: {
    width: 56, height: 56, borderRadius: 28, backgroundColor: "#EEF2FF",
    justifyContent: "center", alignItems: "center", marginBottom: 20,
  },
  questionText: { fontSize: 24, fontWeight: "800", color: "#111827", textAlign: "center", marginBottom: 12, lineHeight: 32 },
  questionSubtext: { fontSize: 16, color: "#64748B", textAlign: "center", lineHeight: 20 },
  optionsContainer: { gap: 8 },
  optionButton: {
    borderRadius: 20, borderWidth: 2, borderColor: "#E2E8F0", backgroundColor: "#FFFFFF",
    overflow: "hidden", height: 110, marginBottom: 8,
  },
  optionButtonSelected: {
    borderWidth: 2, shadowColor: "#3B82F6", shadowOpacity: 0.15, shadowOffset: { width: 0, height: 4 },
    shadowRadius: 12, elevation: 6, transform: [{ scale: 1.02 }],
  },
  optionContent: { flexDirection: "row", alignItems: "center", padding: 20 },
  emojiContainer: {
    width: 48, height: 48, borderRadius: 24, justifyContent: "center", alignItems: "center", marginRight: 16,
  },
  emoji: { fontSize: 22 },
  optionTextContainer: { flex: 1 },
  optionLabel: { fontSize: 17, fontWeight: "600", color: "#111827", marginBottom: 6, lineHeight: 24 },
  optionDescription: { fontSize: 14, color: "#64748B", lineHeight: 20 },
  checkContainer: { width: 28, height: 28, borderRadius: 14, justifyContent: "center", alignItems: "center", marginLeft: 12 },
});
