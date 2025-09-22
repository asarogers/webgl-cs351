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
import { QUIZ as quiz_options } from "../../../components/Interests_lifestyle"; // ⬅️ keyed schema

const { width } = Dimensions.get("window");

// grab the schema question
const q = quiz_options.sections.lifestyle.questions.guest_frequency;

// optional UI metadata to enrich simple labels
/** @type {{ [key: string]: { icon: string; color: string; description: string } }} */
const FRIENDS_META = {
  rare:   { icon: "🏡", color: "#10B981", description: "Quiet nights in are my preference" },
  weekly: { icon: "🧑‍🤝‍🧑", color: "#3B82F6", description: "Regular friend gatherings" },
  often:  { icon: "🎉", color: "#F59E0B", description: "Love hosting and socializing" },
};

// build the options the UI expects
const FRIEND_OPTIONS = q.optionsOrder.map((id) => ({
  id,
  label: q.options[id].label,
  icon: FRIENDS_META[id]?.icon ?? "👥",
  color: FRIENDS_META[id]?.color ?? "#3B82F6",
  description: FRIENDS_META[id]?.description ?? "",
}));

export default function FriendsOverQuestion({ selected, onSelect, onNext }) {
  const [buttonAnimations] = useState(
    FRIEND_OPTIONS.map(() => new Animated.Value(0))
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
    if (selected) {
      Animated.spring(selectionAnimation, {
        toValue: 1,
        tension: 100,
        friction: 8,
        useNativeDriver: true,
      }).start();
    }
  }, [selected]);

  const handleSelect = (id) => {
    onSelect?.(id);
    const i = FRIEND_OPTIONS.findIndex((opt) => opt.id === id);
    if (i !== -1) {
      Animated.sequence([
        Animated.timing(buttonAnimations[i], {
          toValue: 0.95,
          duration: 100,
          useNativeDriver: true,
        }),
        Animated.timing(buttonAnimations[i], {
          toValue: 1,
          duration: 100,
          useNativeDriver: true,
        }),
      ]).start();
    }
  };

  return (
    <SafeAreaView style={styles.safeArea}>
      <View style={styles.card}>
        <View style={styles.questionHeader}>
          <View style={styles.questionIcon}>
            <Ionicons name="people" size={24} color="#3B82F6" />
          </View>
          {/* Pull copy from schema if you add it later; for now keep your text */}
          <Text style={styles.questionText}>How often do you have friends over?</Text>
          <Text style={styles.questionSubtext}>
            Help us match you with compatible social preferences
          </Text>
        </View>

        <View style={styles.optionsContainer}>
          {FRIEND_OPTIONS.map((opt, idx) => {
            const isSelected = selected === opt.id;
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
                    isSelected && [
                      styles.optionBtnSelected,
                      { borderColor: opt.color, backgroundColor: `${opt.color}10` },
                    ],
                  ]}
                  onPress={() => handleSelect(opt.id)}
                  activeOpacity={0.8}
                >
                  <View style={styles.optionContent}>
                    <View
                      style={[
                        styles.emojiContainer,
                        { backgroundColor: `${opt.color}20` },
                        isSelected && { backgroundColor: `${opt.color}30`, transform: [{ scale: 1.1 }] },
                      ]}
                    >
                      <Text style={styles.optionIcon}>{opt.icon}</Text>
                    </View>

                    <View style={styles.optionTextContainer}>
                      <Text
                        style={[
                          styles.optionLabel,
                          isSelected && { color: opt.color, fontWeight: "700" },
                        ]}
                      >
                        {opt.label}
                      </Text>
                      {!!opt.description && (
                        <Text
                          style={[
                            styles.optionDescription,
                            isSelected && { color: "#374151" },
                          ]}
                        >
                          {opt.description}
                        </Text>
                      )}
                    </View>

                    {isSelected && (
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
    marginTop: 0, backgroundColor: "white", borderRadius: 24, padding: 10,
    shadowColor: "#000", shadowOpacity: 0.1, shadowOffset: { width: 0, height: 8 },
    shadowRadius: 24, elevation: 12, marginHorizontal: 0, flex: 1, justifyContent: "center",
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
