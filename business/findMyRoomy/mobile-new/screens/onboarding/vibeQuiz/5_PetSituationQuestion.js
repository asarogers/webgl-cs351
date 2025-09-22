import React, { useState, useEffect } from "react";
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
import { QUIZ as quiz_options } from "../../../components/Interests_lifestyle";

const { width } = Dimensions.get("window");

// --- pull the correct question from the keyed schema ---
const q = quiz_options.sections.pets.questions.pet_ownership;

/** @type {{ [key: string]: { icon: string; color: string; description: string } }} */
const PET_META = {
  dog:       { icon: "🐕", color: "#F59E0B", description: "Well-trained and house-broken" },
  cat:       { icon: "🐈", color: "#8B5CF6", description: "Indoor cat, litter box maintained" },
  small_pet: { icon: "🐹", color: "#10B981", description: "Birds, hamsters, fish, reptile, etc." },
  other:     { icon: "❤️", color: "#EF4444", description: "Something else? Tell us what." },
  none:      { icon: "🚫", color: "#6B7280", description: "No pets currently" },
};

// Build UI options array from schema
const PET_OPTIONS = q.optionsOrder.map((id) => ({
  id,
  label: q.options[id].label,
  icon: PET_META[id]?.icon ?? "🐾",
  color: PET_META[id]?.color ?? "#3B82F6",
  description: PET_META[id]?.description ?? "",
}));

export default function PetSituationQuestion({ selected, onSelect, onNext }) {
  // selected can be string or array; normalize helpers:
  const asArray = Array.isArray(selected) ? selected : (selected ? [selected] : []);
  const isSelected = (id) => asArray.includes(id);

  const toggle = (id) => {
    // handle exclusive_with rule in schema: "none" excludes others
    let next = [...asArray];
    if (id === "none") {
      next = isSelected("none") ? [] : ["none"];
    } else {
      // remove "none" if present, then toggle id
      next = next.filter((x) => x !== "none");
      if (isSelected(id)) {
        next = next.filter((x) => x !== id);
      } else {
        next.push(id);
      }
    }
    // if parent expects single value (string), keep parity with input type
    const returnValue = Array.isArray(selected) ? next : next[0] ?? null;
    onSelect?.(returnValue);
  };

  const [buttonAnimations] = useState(
    PET_OPTIONS.map(() => new Animated.Value(0))
  );
  const [selectionAnimation] = useState(new Animated.Value(0));

  useEffect(() => {
    const animations = buttonAnimations.map((value, index) =>
      Animated.timing(value, {
        toValue: 1,
        duration: 400,
        delay: index * 100,
        useNativeDriver: true,
      })
    );
    Animated.stagger(100, animations).start();
  }, []);

  useEffect(() => {
    if (asArray.length) {
      Animated.spring(selectionAnimation, {
        toValue: 1,
        tension: 100,
        friction: 8,
        useNativeDriver: true,
      }).start();
    }
  }, [selected]);

  return (
    <SafeAreaView style={styles.safeArea}>
      <View style={styles.card}>
        <View style={styles.questionHeader}>
          <View style={styles.questionIcon}>
            <Ionicons name="paw" size={24} color="#3B82F6" />
          </View>
          <Text style={styles.questionText}>What’s your pet situation?</Text>
          <Text style={styles.questionSubtext}>
            Select all that apply — we’ll match you with compatible pet preferences
          </Text>
        </View>

        <View style={styles.optionsContainer}>
          {PET_OPTIONS.map((opt, idx) => {
            const selectedNow = isSelected(opt.id);
            return (
              <Animated.View
                key={opt.id}
                style={{
                  opacity: buttonAnimations[idx],
                  transform: [
                    {
                      translateY: buttonAnimations[idx].interpolate({
                        inputRange: [0, 1],
                        outputRange: [20, 0],
                      }),
                    },
                    { scale: buttonAnimations[idx] },
                  ],
                }}
              >
                <TouchableOpacity
                  style={[
                    styles.optionBtn,
                    selectedNow && [
                      styles.optionBtnSelected,
                      { borderColor: opt.color, backgroundColor: `${opt.color}10` },
                    ],
                  ]}
                  onPress={() => toggle(opt.id)}
                  activeOpacity={0.8}
                >
                  <View style={styles.optionContent}>
                    <View
                      style={[
                        styles.emojiContainer,
                        { backgroundColor: `${opt.color}20` },
                        selectedNow && {
                          backgroundColor: `${opt.color}30`,
                          transform: [{ scale: 1.1 }],
                        },
                      ]}
                    >
                      <Text style={styles.optionIcon}>{opt.icon}</Text>
                    </View>

                    <View style={styles.optionTextContainer}>
                      <Text
                        style={[
                          styles.optionLabel,
                          selectedNow && { color: opt.color, fontWeight: "700" },
                        ]}
                      >
                        {opt.label}
                      </Text>
                      {!!opt.description && (
                        <Text
                          style={[
                            styles.optionDescription,
                            selectedNow && { color: "#374151" },
                          ]}
                        >
                          {opt.description}
                        </Text>
                      )}
                    </View>

                    {selectedNow && (
                      <Animated.View
                        style={[
                          styles.checkContainer,
                          { backgroundColor: opt.color },
                          {
                            opacity: selectionAnimation,
                            transform: [
                              {
                                scale: selectionAnimation.interpolate({
                                  inputRange: [0, 1],
                                  outputRange: [0.5, 1],
                                }),
                              },
                            ],
                          },
                        ]}
                      >
                        <Ionicons name="checkmark" size={16} color="white" />
                      </Animated.View>
                    )}
                  </View>
                </TouchableOpacity>
              </Animated.View>
            );
          })}
        </View>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: "#F8FAFC" },
  card: {
    marginTop: 0,
    backgroundColor: "white",
    borderRadius: 24,
    padding: 10,
    shadowColor: "#000",
    shadowOpacity: 0.1,
    shadowOffset: { width: 0, height: 8 },
    shadowRadius: 24,
    elevation: 12,
    marginHorizontal: 0,
    flex: 1,
    justifyContent: "center",
  },
  questionHeader: { alignItems: "center", marginBottom: 32 },
  questionIcon: {
    width: 56, height: 56, borderRadius: 28, backgroundColor: "#EEF2FF",
    justifyContent: "center", alignItems: "center", marginBottom: 20,
  },
  questionText: {
    fontSize: 24, fontWeight: "800", color: "#111827", textAlign: "center",
    marginBottom: 12, lineHeight: 32,
  },
  questionSubtext: { fontSize: 16, color: "#64748B", textAlign: "center", lineHeight: 20 },
  optionsContainer: { gap: 8 },
  optionBtn: {
    borderRadius: 20, borderWidth: 2, borderColor: "#E2E8F0", backgroundColor: "#FFFFFF",
    overflow: "hidden", marginBottom: 8,
  },
  optionBtnSelected: {
    borderWidth: 2, shadowColor: "#3B82F6", shadowOpacity: 0.15, shadowOffset: { width: 0, height: 4 },
    shadowRadius: 12, elevation: 6, transform: [{ scale: 1.02 }],
  },
  optionContent: { flexDirection: "row", alignItems: "center", padding: 20 },
  emojiContainer: {
    width: 48, height: 48, borderRadius: 24, justifyContent: "center", alignItems: "center", marginRight: 16,
  },
  optionIcon: { fontSize: 22 },
  optionTextContainer: { flex: 1 },
  optionLabel: { fontSize: 17, fontWeight: "600", color: "#111827", marginBottom: 6, lineHeight: 24 },
  optionDescription: { fontSize: 14, color: "#64748B", lineHeight: 20 },
  checkContainer: {
    width: 28, height: 28, borderRadius: 14, justifyContent: "center", alignItems: "center", marginLeft: 12,
  },
});
