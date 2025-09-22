import React, { useEffect, useRef, useState } from "react";
import { Dimensions, FlatList } from "react-native";
import { housing_preferences } from "../../../../../components/Interests_lifestyle";
import { getStepByNumber } from "../../../../../components/onboardingStepsHelper";
import authService from "../../../../../database/authService";
import ZoneDrawingAnimation  from "../ZoneDrawingAnimation";
import QuizIntro from "./1_QuizIntro";
import DailyHabits from "./2_DailyHabits";
import DishesQuestion from "./3_DishesQuestion";
import FriendsOverQuestion from "./4_FriendsOverQuestion";
import PetSituationQuestion from "./5_PetSituationQuestion";
import MoveInTimelineQuestion from "./6_MoveInTimelineQuestion";
import LeaseDurationQuestion from "./7_LeaseDurationQuestion";
import BudgetRangeQuestion from "./8_BudgetRangeQuestion";
import ProgressBar from "./Progressbar";

const { width } = Dimensions.get("window");

// Each quiz key mapped to its DB field name
const FIELD_MAP = {
  intro: "weekend_vibe",
  habits: "sleep_schedule",
  dishes: "dish_washing",
  friends: "friends_over",
  pets: "pet_situation",
  movein: "move_in_selection",
  lease: "lease_duration",
  draw: "zone_drawn",
};

const slides = [
  { key: "intro" },
  { key: "habits" },
  { key: "dishes" },
  { key: "friends" },
  { key: "pets" },
  { key: "movein" },
  { key: "lease" },
  { key: "budget" },
  { key: "draw" },
];

export default function LifestyleQuizCarousel({ navigation, route }){
  const { givenStep } = route.params || {};
  const [selections, setSelections] = useState({});
  const [isSaving, setIsSaving] = useState(false);
  const flatListRef = useRef(null);
  const [currentIndex, setCurrentIndex] = useState(0);
  const isScrollingProgrammatically = useRef(false);

  useEffect(() => {
    let isMounted = true;
    (async () => {
      const answers = await authService.getQuizAnswers();
      if (isMounted && answers) {
        const prefill = {};

        // Loop through standard field mapping
        Object.entries(FIELD_MAP).forEach(([quizKey, dbField]) => {
          if (answers[dbField] != null) {
            prefill[quizKey] = answers[dbField];
          }
        });

        // Handle special cases
        if (answers.move_in_selection) {
          if (/^\d{4}-\d{2}-\d{2}$/.test(answers.move_in_selection)) {
            prefill.moveinChoice = "specific_date";
            prefill.moveinDate = new Date(answers.move_in_selection);
          } else {
            prefill.moveinChoice = answers.move_in_selection;
          }
        }

        if (
          typeof answers.budget_min === "number" &&
          typeof answers.budget_max === "number"
        ) {
          const match = housing_preferences.budget_range.find(
            (opt) =>
              opt.range &&
              opt.range[0] === answers.budget_min &&
              opt.range[1] === answers.budget_max
          );

          if (match) {
            prefill.budget = match.id;
          } else {
            prefill.budget = "custom_budget";
            prefill.customBudgetRange = [
              answers.budget_min,
              answers.budget_max,
            ];
          }
        }

        setSelections(prefill);
      }
    })();
    return () => {
      isMounted = false;
    };
  }, []);

  useEffect(() => {
    const stepToKey = {
      2: "intro",
      3: "habits",
      4: "dishes",
      5: "friends",
      6: "pets",
      7: "movein",
      8: "lease",
      9: "budget",
      10: "draw",
    };

    if (givenStep) {
      console.log("given step", givenStep);
      const key = stepToKey[Number(givenStep)] || "intro";
      const idx = slides.findIndex((s) => s.key === key);
      setCurrentIndex(idx);
      setTimeout(() => {
        flatListRef.current?.scrollToIndex({ index: idx, animated: false });
      }, 50);
    }
  }, [givenStep]);

  // Save to DB, then advance to next
  const saveAndGoNext = async () => {
    const slideKey = slides[currentIndex].key;
    const dbField = FIELD_MAP[slideKey];

    let payload = {};

    if (slideKey === "movein") {
      payload = {
        [dbField]:
          selections.moveinChoice === "specific_date"
            ? selections.moveinDate?.toISOString().split("T")[0]
            : selections.moveinChoice,
      };
    } else if (slideKey === "budget") {
      const selectedBudget = selections.budget; // can be [min,max] or an id string

      payload = {};

      // ✅ If it's an array from the slider, save it directly
      if (Array.isArray(selectedBudget) && selectedBudget.length === 2) {
        payload.budget_min = selectedBudget[0];
        payload.budget_max = selectedBudget[1];
      } else {
        // original ID-based logic
        const selectedOption = housing_preferences.budget_range.find(
          (opt) => opt.id === selectedBudget
        );

        if (
          selectedBudget === "custom_budget" &&
          selections.customBudgetRange
        ) {
          payload.budget_min = selections.customBudgetRange[0];
          payload.budget_max = selections.customBudgetRange[1];
        } else if (selectedOption?.range) {
          const [min, max] = selectedOption.range;
          payload.budget_min = min;
          payload.budget_max = max;
        } else {
          payload.budget_min = null;
          payload.budget_max = null;
        }
      }
    } else if (dbField && selections[slideKey]) {
      payload = {
        [dbField]: selections[slideKey],
      };
    }

    const onboarding = getStepByNumber(currentIndex + 4);
    await authService.setOnboardingStep(onboarding);

    if (Object.keys(payload).length > 0) {
      setIsSaving(true);
      await authService.saveQuizAnswer(payload);
      setIsSaving(false);
    }

    if (currentIndex < slides.length - 1) {
      const nextIdx = currentIndex + 1;
      isScrollingProgrammatically.current = true;
      setCurrentIndex(nextIdx);
      setTimeout(() => {
        flatListRef.current?.scrollToIndex({ index: nextIdx, animated: true });
        setTimeout(() => {
          isScrollingProgrammatically.current = false;
        }, 500);
      }, 10);
    } else {
      navigation.replace("RequestLocation");
    }
  };

  const saveAndGoBack = async () => {
    // Optionally save when going back as well (often not necessary)
    if (currentIndex > 0) {
      const slideKey = slides[currentIndex].key;

      let answer = selections[slideKey];
      if (slideKey === "movein") {
        answer =
          selections.moveinChoice === "specific_date"
            ? selections.moveinDate
            : selections.moveinChoice;
      }
      const dbField = FIELD_MAP[slideKey];

      if (dbField && answer) {
        setIsSaving(true);
        await authService.saveQuizAnswer(dbField, answer);
        setIsSaving(false);
      }

      const nextIdx = currentIndex - 1;
      isScrollingProgrammatically.current = true;
      setCurrentIndex(nextIdx);
      setTimeout(() => {
        flatListRef.current?.scrollToIndex({ index: nextIdx, animated: true });
        setTimeout(() => {
          isScrollingProgrammatically.current = false;
        }, 500);
      }, 10);
    } else {
      navigation.replace("UserIntent");
    }
  };

  const goToIndex = (index) => {
    if (index >= 0 && index < slides.length) {
      isScrollingProgrammatically.current = true;
      setCurrentIndex(index);
      setTimeout(() => {
        flatListRef.current?.scrollToIndex({ index, animated: true });
        setTimeout(() => {
          isScrollingProgrammatically.current = false;
        }, 500);
      }, 10);
    }
  };

  const goSkip = () => navigation.replace("UserIntent");

  const handleSelect = (slideKey, value) => {
    setSelections((prev) => ({
      ...prev,
      [slideKey]: value,
    }));
  };

  const renderItem = ({ item, index }) => {
    const slideKey = item.key;
    const selected = selections[slideKey];
    const canProceed =
      slideKey === "movein"
        ? !!selections.moveinChoice &&
          (selections.moveinChoice !== "specific_date" ||
            !!selections.moveinDate)
        : slideKey === "budget"
        ?
          (Array.isArray(selections.budget) &&
            selections.budget.length === 2 &&
            selections.budget.every((n) => typeof n === "number")) ||

          (!!selections.budget &&
            (selections.budget !== "custom_budget" ||
              (Array.isArray(selections.customBudgetRange) &&
                selections.customBudgetRange.length === 2 &&
                selections.customBudgetRange.every(
                  (n) => typeof n === "number"
                ))))
        : !!selected;

    const commonProps = {
      onNext: saveAndGoNext,
      onSkip: goSkip,
      setIndex: goToIndex,
      index: currentIndex,
      width,
      onSelect: (value) => handleSelect(slideKey, value),
      selected,
      isSaving,
    };

    let content;
    switch (slideKey) {
      case "intro":
        content = <QuizIntro {...commonProps} />;
        break;
      case "habits":
        content = <DailyHabits {...commonProps} />;
        break;
      case "dishes":
        content = <DishesQuestion {...commonProps} />;
        break;
      case "friends":
        content = <FriendsOverQuestion {...commonProps} />;
        break;
      case "pets":
        content = <PetSituationQuestion {...commonProps} />;
        break;

      case "movein":
        content = (
          <MoveInTimelineQuestion
            {...commonProps}
            selected={selections.moveinChoice}
            onSelect={(v) => handleSelect("moveinChoice", v)}
            specificDate={selections.moveinDate}
            onDateSelect={(d) => handleSelect("moveinDate", d)}
          />
        );
        break;
      case "lease":
        content = <LeaseDurationQuestion {...commonProps} />;
        break;
      case "budget":
        content = (
          <BudgetRangeQuestion
            {...commonProps}
            selected={selections.budget}
            onSelect={(value) => handleSelect("budget", value)}
            customBudgetRange={selections.customBudgetRange}
            onBudgetRangeChange={(range) =>
              handleSelect("customBudgetRange", range)
            }
          />
        );
        break;
      case "draw":
        content = <ZoneDrawingAnimation {...commonProps} />;
        break;
      default:
        return null;
    }

    return (
      <ProgressBar
        currentStep={index + 1}
        totalSteps={slides.length}
        onSkip={goSkip}
        onNext={saveAndGoNext}
        goBack={saveAndGoBack}
        canProceed={canProceed}
        navigation={navigation}
        isSaving={isSaving}
      >
        {content}
      </ProgressBar>
    );
  };

  const handleScroll = (e) => {
    if (!isScrollingProgrammatically.current) {
      const idx = Math.round(e.nativeEvent.contentOffset.x / width);
      if (idx !== currentIndex) {
        setCurrentIndex(idx);
      }
    }
  };

  return (
    <FlatList
      ref={flatListRef}
      data={slides}
      horizontal
      pagingEnabled
      showsHorizontalScrollIndicator={false}
      renderItem={renderItem}
      keyExtractor={(item) => item.key}
      onScroll={handleScroll}
      scrollEventThrottle={16}
      scrollEnabled={false}
      extraData={{ selections, isSaving }}
    />
  );
};
