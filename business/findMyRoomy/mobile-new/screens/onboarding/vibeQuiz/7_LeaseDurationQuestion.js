import React, { useEffect, useMemo, useState } from "react";
import {
  View,
  Text,
  TouchableOpacity,
  StyleSheet,
  Animated,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { Ionicons } from "@expo/vector-icons";
import { QUIZ as quiz_options } from "../../../components/Interests_lifestyle"; 

// Grab the question from schema
const q = quiz_options.sections.basics.questions.lease_duration;

/** @type {{[k:string]: {icon:string, color:string, description:string}}} */
const META = {
  "3_6": { icon: "📋", color: "#F59E0B", description: "Temporary housing or trying out the area" },
  "12":  { icon: "📄", color: "#10B981", description: "Most common rental period" },
  "18p": { icon: "🏠", color: "#3B82F6", description: "Settling down for a while" },
  mtm:   { icon: "🔄", color: "#8B5CF6", description: "Maximum flexibility" },
  flex:  { icon: "🤷‍♀️", color: "#6B7280", description: "Open to discussing with roommates" },
};

export default function LeaseDurationQuestion({ selected, onSelect }) {
  const [fadeAnim] = useState(new Animated.Value(0));
  const [slideAnim] = useState(new Animated.Value(30));

  // Build UI options from schema (keeps labels in one source of truth)
  const options = useMemo(
    () =>
      q.optionsOrder.map((id) => ({
        id,
        label: q.options[id].label,
        icon: META[id]?.icon ?? "📄",
        color: META[id]?.color ?? "#3B82F6",
        description: META[id]?.description ?? "",
      })),
    []
  );

  const [buttonAnimations] = useState(options.map(() => new Animated.Value(0)));

  useEffect(() => {
    Animated.parallel([
      Animated.timing(fadeAnim, { toValue: 1, duration: 600, useNativeDriver: true }),
      Animated.timing(slideAnim, { toValue: 0, duration: 600, useNativeDriver: true }),
    ]).start();

    const seq = buttonAnimations.map((anim, i) =>
      Animated.timing(anim, { toValue: 1, duration: 400, delay: i * 100, useNativeDriver: true })
    );
    Animated.stagger(100, seq).start();
  }, []);

  const handleSelect = (id) => onSelect?.(id);

  return (
    <SafeAreaView style={styles.container}>
      <Animated.View style={[styles.content, { opacity: fadeAnim, transform: [{ translateY: slideAnim }] }]}>
        <View style={styles.card}>
          <View style={styles.questionHeader}>
            <View style={styles.questionIcon}>
              <Ionicons name="contract-outline" size={24} color="#3B82F6" />
            </View>
            <Text style={styles.questionText}>What lease length works for you?</Text>
            <Text style={styles.questionSubtext}>Different roommates have different timeline needs</Text>
          </View>

          <View style={styles.optionsContainer}>
            {options.map((opt, index) => (
              <Animated.View
                key={opt.id}
                style={{
                  opacity: buttonAnimations[index],
                  transform: [
                    {
                      translateY: buttonAnimations[index].interpolate({
                        inputRange: [0, 1],
                        outputRange: [20, 0],
                      }),
                    },
                    { scale: buttonAnimations[index] },
                  ],
                }}
              >
                <TouchableOpacity
                  onPress={() => handleSelect(opt.id)}
                  style={[
                    styles.optionButton,
                    selected === opt.id && [
                      styles.optionButtonSelected,
                      { borderColor: opt.color, backgroundColor: `${opt.color}10` },
                    ],
                  ]}
                  activeOpacity={0.8}
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
      </Animated.View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { backgroundColor: "#F8FAFC", flex: 1 },
  content: { flex: 1, paddingHorizontal: 0, paddingTop: 0, justifyContent: "center" },
  card: {
    marginTop: 0, backgroundColor: "white", borderRadius: 24, padding: 10,
    shadowColor: "#000", shadowOpacity: 0.1, shadowOffset: { width: 0, height: 8 },
    shadowRadius: 24, elevation: 12, marginHorizontal: 0, maxHeight: "85%",
  },
  questionHeader: { alignItems: "center", marginBottom: 32 },
  questionIcon: {
    width: 56, height: 56, borderRadius: 28, backgroundColor: "#EEF2FF",
    justifyContent: "center", alignItems: "center", marginBottom: 20,
  },
  questionText: { fontSize: 24, fontWeight: "800", color: "#111827", textAlign: "center", marginBottom: 12, lineHeight: 32 },
  questionSubtext: { fontSize: 16, color: "#64748B", textAlign: "center", lineHeight: 20 },
  optionsContainer: { gap: 8, paddingBottom: 20 },
  optionButton: {
    borderRadius: 20, borderWidth: 2, borderColor: "#E2E8F0", backgroundColor: "#FFFFFF",
    overflow: "hidden", minHeight: 88,
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
  checkContainer: {
    width: 28, height: 28, borderRadius: 14, justifyContent: "center", alignItems: "center", marginLeft: 12,
  },
});
