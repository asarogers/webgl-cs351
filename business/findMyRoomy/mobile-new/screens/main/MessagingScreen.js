import React, { useState, useEffect } from "react";
import {
  View,
  StyleSheet,

  Alert,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { CometChat } from "@cometchat-pro/react-native-chat";
import {
  CometChatConversationList,
  CometChatMessages,
  CometChatConversationListWithMessages,
} from "@cometchat-pro/react-native-chat-ui-kit";

const colors = {
  primary: "#3B82F6",
  error: "#EF4444",
  navy: "#1F2937",
  slate: "#374151",
  grayMedium: "#6B7280",
  grayLight: "#E5E7EB",
  background: "#F8FAFC",
  card: "#FFFFFF",
  green: "#10B981",
};

export const MessagesScreen = ({ setPage, currentUser }) => {
  const [selectedConversation, setSelectedConversation] = useState(null);
  const [isInitialized, setIsInitialized] = useState(false);

  useEffect(() => {
    initializeCometChat();
  }, []);

  const initializeCometChat = async () => {
    try {
      // Initialize CometChat if not already initialized
      const isInitialized = await CometChat.isInitialized();
      if (!isInitialized) {
        const appSetting = new CometChat.AppSettingsBuilder()
          .subscribePresenceForAllUsers()
          .setRegion("YOUR_REGION") // Replace with your region
          .build();
        
        await CometChat.init("YOUR_APP_ID", appSetting); // Replace with your App ID
      }

      // Login user if not already logged in
      const user = await CometChat.getLoggedinUser();
      if (!user && currentUser) {
        const authKey = "YOUR_AUTH_KEY"; // Replace with your Auth Key
        await CometChat.login(currentUser.uid, authKey);
      }
      
      setIsInitialized(true);
    } catch (error) {
      console.error("CometChat initialization failed:", error);
      Alert.alert("Error", "Failed to initialize chat. Please try again.");
    }
  };

  const handleConversationClick = (conversation) => {
    setSelectedConversation(conversation);
  };

  const handleBackPress = () => {
    setSelectedConversation(null);
  };

  if (!isInitialized) {
    return (
      <SafeAreaView style={styles.container}>
        <View style={styles.loadingContainer}>
          {/* You can add a loading spinner here */}
        </View>
      </SafeAreaView>
    );
  }

  // If a conversation is selected, show the messages screen
  if (selectedConversation) {
    return (
      <SafeAreaView style={styles.container}>
        <CometChatMessages
          item={selectedConversation}
          type={selectedConversation.conversationType}
          theme={{
            palette: {
              mode: "light",
              primary: {
                light: colors.primary,
                main: colors.primary,
                dark: colors.primary,
              },
              background: {
                default: colors.background,
                paper: colors.card,
              },
              text: {
                primary: colors.navy,
                secondary: colors.grayMedium,
              },
            },
            typography: {
              title1: {
                fontSize: 18,
                fontWeight: "700",
                color: colors.navy,
              },
              body: {
                fontSize: 15,
                color: colors.navy,
              },
            },
          }}
          messageHeaderConfiguration={{
            showBackButton: true,
            onBackPress: handleBackPress,
            hideDetails: false,
          }}
          messageListConfiguration={{
            showAvatar: true,
            showReactions: true,
            showReceipt: true,
            dateFormat: "MMM DD, YYYY",
            timeFormat: "HH:mm A",
          }}
          messageComposerConfiguration={{
            attachmentOptions: [
              CometChat.ATTACHMENT_TYPE.FILE,
              CometChat.ATTACHMENT_TYPE.IMAGE,
              CometChat.ATTACHMENT_TYPE.VIDEO,
              CometChat.ATTACHMENT_TYPE.AUDIO,
            ],
            showSendButton: true,
            placeholder: "Type a message...",
          }}
        />
      </SafeAreaView>
    );
  }

  // Show conversation list
  return (
    <SafeAreaView style={styles.container}>
      <CometChatConversationList
        onItemPress={handleConversationClick}
        searchBoxConfiguration={{
          placeholder: "Search messages",
          showSearchBox: true,
        }}
        listItemConfiguration={{
          showAvatar: true,
          showLastMessage: true,
          showUnreadCount: true,
          showDeliveryReceipt: true,
          dateFormat: "MMM DD",
          timeFormat: "HH:mm A",
        }}
        theme={{
          palette: {
            mode: "light",
            primary: {
              light: colors.primary,
              main: colors.primary,
              dark: colors.primary,
            },
            background: {
              default: colors.background,
              paper: colors.card,
            },
            text: {
              primary: colors.navy,
              secondary: colors.grayMedium,
            },
            divider: colors.grayLight,
          },
          typography: {
            title1: {
              fontSize: 16,
              fontWeight: "700",
              color: colors.navy,
            },
            subtitle1: {
              fontSize: 13,
              color: colors.grayMedium,
            },
            caption: {
              fontSize: 12,
              color: colors.grayMedium,
            },
          },
          spacing: {
            padding: 16,
            margin: 8,
          },
        }}
        emptyStateText="No conversations found"
        errorStateText="Something went wrong"
        loadingIconTint={colors.primary}
      />
    </SafeAreaView>
  );
};

// Alternative: Use the combined component for a more integrated experience
export const MessagesScreenCombined = ({ setPage, currentUser }) => {
  const [isInitialized, setIsInitialized] = useState(false);

  useEffect(() => {
    initializeCometChat();
  }, []);

  const initializeCometChat = async () => {
    try {
      const isInitialized = await CometChat.isInitialized();
      if (!isInitialized) {
        const appSetting = new CometChat.AppSettingsBuilder()
          .subscribePresenceForAllUsers()
          .setRegion("YOUR_REGION")
          .build();
        
        await CometChat.init("YOUR_APP_ID", appSetting);
      }

      const user = await CometChat.getLoggedinUser();
      if (!user && currentUser) {
        const authKey = "YOUR_AUTH_KEY";
        await CometChat.login(currentUser.uid, authKey);
      }
      
      setIsInitialized(true);
    } catch (error) {
      console.error("CometChat initialization failed:", error);
      Alert.alert("Error", "Failed to initialize chat. Please try again.");
    }
  };

  if (!isInitialized) {
    return (
      <SafeAreaView style={styles.container}>
        <View style={styles.loadingContainer}>
          {/* Loading state */}
        </View>
      </SafeAreaView>
    );
  }

  return (
    <SafeAreaView style={styles.container}>
      <CometChatConversationListWithMessages
        conversationListConfiguration={{
          searchBoxConfiguration: {
            placeholder: "Search messages",
            showSearchBox: true,
          },
          listItemConfiguration: {
            showAvatar: true,
            showLastMessage: true,
            showUnreadCount: true,
            showDeliveryReceipt: true,
            dateFormat: "MMM DD",
            timeFormat: "HH:mm A",
          },
        }}
        messageConfiguration={{
          messageHeaderConfiguration: {
            showBackButton: true,
            hideDetails: false,
          },
          messageListConfiguration: {
            showAvatar: true,
            showReactions: true,
            showReceipt: true,
            dateFormat: "MMM DD, YYYY",
            timeFormat: "HH:mm A",
          },
          messageComposerConfiguration: {
            attachmentOptions: [
              CometChat.ATTACHMENT_TYPE.FILE,
              CometChat.ATTACHMENT_TYPE.IMAGE,
              CometChat.ATTACHMENT_TYPE.VIDEO,
              CometChat.ATTACHMENT_TYPE.AUDIO,
            ],
            showSendButton: true,
            placeholder: "Type a message...",
          },
        }}
        theme={{
          palette: {
            mode: "light",
            primary: {
              light: colors.primary,
              main: colors.primary,
              dark: colors.primary,
            },
            background: {
              default: colors.background,
              paper: colors.card,
            },
            text: {
              primary: colors.navy,
              secondary: colors.grayMedium,
            },
            divider: colors.grayLight,
          },
          typography: {
            title1: {
              fontSize: 18,
              fontWeight: "700",
              color: colors.navy,
            },
            body: {
              fontSize: 15,
              color: colors.navy,
            },
            subtitle1: {
              fontSize: 13,
              color: colors.grayMedium,
            },
            caption: {
              fontSize: 12,
              color: colors.grayMedium,
            },
          },
        }}
      />
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: colors.background,
  },
  loadingContainer: {
    flex: 1,
    justifyContent: "center",
    alignItems: "center",
    backgroundColor: colors.background,
  },
});

export default MessagesScreen;