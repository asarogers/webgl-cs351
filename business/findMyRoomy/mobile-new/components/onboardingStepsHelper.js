// onboardingStepsHelper.ts

export const onboardingOrder = [
  "not_started",
  "verified",
  "onboarding_1",
  "onboarding_2",
  "onboarding_3",
  "onboarding_4",
  "onboarding_5",
  "onboarding_6",
  "onboarding_7",
  "onboarding_8",
  "onboarding_9",
  "request_location",
  "profile_completed",
];

export function getStepByNumber(n) {
  return onboardingOrder[n] || null;
}

export async function navigateTo(onboarding, navigation) {
  switch (onboarding) {
    case "not_started":
    case null:
      navigation.navigate("PhoneAndEmailVerificationScreen");
      return;

    case "verified":
      navigation.navigate("Onboarding", { screen: "UserIntent" });
      return;

    case "request_location":
      navigation.replace("Onboarding", { screen: "RequestLocation" });
      return;

    case "profile_completed":
      navigation.replace("Main", { screen: "Map" });
      return;

    default: {
      // Handle any onboarding_N by routing into Quiz at the right slide.
      // Your carousel interprets `givenStep` so that:
      //   givenStep 2 -> intro (index 0)
      //   givenStep 3 -> habits (index 1)
      //   ...
      //   givenStep 11 -> draw (index 9)
      if (typeof onboarding === "string" && onboarding.startsWith("onboarding_")) {
        const n = Number(onboarding.split("_")[1]); // e.g. "onboarding_7" -> 7
        // Convert onboarding_N -> givenStep = N - 1 (bounded to >= 2)
        const givenStep = String(Math.max(2, n));
        navigation.navigate("Onboarding", {
          screen: "Quiz",
          params: { givenStep },
        });
        return;
      }
      console.log("no onboarding step found");
    }
  }
}
