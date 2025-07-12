#!/bin/bash

BASE_DIR="src"

# List of all directories to create
dirs=(
  "$BASE_DIR/screens/onboarding"
  "$BASE_DIR/screens/main"
  "$BASE_DIR/screens/housing"
  "$BASE_DIR/screens/settings"
  "$BASE_DIR/components/common"
  "$BASE_DIR/components/onboarding"
  "$BASE_DIR/components/map"
  "$BASE_DIR/components/messaging"
  "$BASE_DIR/navigation"
  "$BASE_DIR/constants"
  "$BASE_DIR/utils"
)

# List of all files in format "directory:filename"
files=(
  "$BASE_DIR/screens/onboarding:SplashScreen.js"
  "$BASE_DIR/screens/onboarding:WelcomeCarousel.js"
  "$BASE_DIR/screens/onboarding:UserIntentScreen.js"
  "$BASE_DIR/screens/onboarding:SignUpScreen.js"
  "$BASE_DIR/screens/onboarding:LoginScreen.js"
  "$BASE_DIR/screens/onboarding:PhoneVerificationScreen.js"
  "$BASE_DIR/screens/onboarding:EmailVerificationScreen.js"
  "$BASE_DIR/screens/onboarding:VibeQuizScreen.js"
  "$BASE_DIR/screens/onboarding:LifestyleQuizScreen.js"
  "$BASE_DIR/screens/onboarding:LocationAccessScreen.js"
  "$BASE_DIR/screens/onboarding:OnboardingTip1Screen.js"
  "$BASE_DIR/screens/onboarding:OnboardingTip2Screen.js"
  "$BASE_DIR/screens/onboarding:OnboardingTip3Screen.js"

  "$BASE_DIR/screens/main:MapScreen.js"
  "$BASE_DIR/screens/main:DiscoveryScreen.js"
  "$BASE_DIR/screens/main:ProfilePreviewScreen.js"
  "$BASE_DIR/screens/main:MessagingScreen.js"
  "$BASE_DIR/screens/main:EmptyAccountScreen.js"

  "$BASE_DIR/screens/housing:JointApplicationStart.js"
  "$BASE_DIR/screens/housing:JointApplicationRoommate.js"
  "$BASE_DIR/screens/housing:JointApplicationConfirm.js"
  "$BASE_DIR/screens/housing:JointApplicationSubmit.js"
  "$BASE_DIR/screens/housing:LeaseAgreementIntro.js"
  "$BASE_DIR/screens/housing:LeaseAgreementUpload.js"

  "$BASE_DIR/screens/settings:TrustSafety.js"
  "$BASE_DIR/screens/settings:SettingsHome.js"
  "$BASE_DIR/screens/settings:NotificationPrefs.js"
  "$BASE_DIR/screens/settings:PrivacySettings.js"
  "$BASE_DIR/screens/settings:ReportProblem.js"
  "$BASE_DIR/screens/settings:SafetyTips.js"
  "$BASE_DIR/screens/settings:DeleteAccount.js"
  "$BASE_DIR/screens/settings:BlockedUsers.js"

  "$BASE_DIR/components/common:Button.js"
  "$BASE_DIR/components/common:TextInput.js"
  "$BASE_DIR/components/common:Avatar.js"
  "$BASE_DIR/components/common:Card.js"
  "$BASE_DIR/components/common:Modal.js"

  "$BASE_DIR/components/onboarding:ProgressDots.js"
  "$BASE_DIR/components/onboarding:OnboardingHeader.js"

  "$BASE_DIR/components/map:ZoneSelector.js"
  "$BASE_DIR/components/map:PinPreviewCard.js"

  "$BASE_DIR/components/messaging:ChatBubble.js"
  "$BASE_DIR/components/messaging:InputBar.js"

  "$BASE_DIR/navigation:AppNavigator.js"
  "$BASE_DIR/navigation:AuthNavigator.js"
  "$BASE_DIR/navigation:OnboardingNavigator.js"
  "$BASE_DIR/navigation:TabNavigator.js"

  "$BASE_DIR/constants:colors.js"
  "$BASE_DIR/constants:spacing.js"
  "$BASE_DIR/constants:typography.js"
  "$BASE_DIR/constants:icons.js"

  "$BASE_DIR/utils:auth.js"
  "$BASE_DIR/utils:api.js"
  "$BASE_DIR/utils:validation.js"
  "$BASE_DIR/utils:analytics.js"
)

# Create directories
for dir in "${dirs[@]}"; do
  mkdir -p "$dir"
done

# Create files with placeholders
for entry in "${files[@]}"; do
  IFS=":" read -r dir filename <<< "$entry"
  filepath="$dir/$filename"
  echo "// $filename" > "$filepath"
done

echo "✅ Fixed structure created successfully with JS files!"
