// book/src/auth/mockAuth.ts
// Mock authentication functions for frontend-only implementation

// Mock user type
export interface MockUser {
  id: string;
  email: string;
  software_background?: string;
  hardware_background?: string;
  created_at?: string;
}

// Mock auth state
let mockAuthState = {
  isAuthenticated: false,
  user: null as MockUser | null,
  isLoading: false,
};

// Mock sign in function
export const mockSignIn = async (email: string, password: string): Promise<{ error?: string; data?: { user: MockUser } }> => {
  // Simulate API call delay
  await new Promise(resolve => setTimeout(resolve, 500));

  // Simple mock validation
  if (email === 'test@example.com' && password === 'Password123') {
    mockAuthState = {
      ...mockAuthState,
      isAuthenticated: true,
      user: {
        id: '1',
        email: 'test@example.com',
        software_background: 'Intermediate',
        hardware_background: 'Hands-on Experience',
        created_at: new Date().toISOString(),
      },
    };
    return { data: { user: mockAuthState.user! } };
  } else {
    return { error: 'Invalid email or password' };
  }
};

// Mock sign up function
export const mockSignUp = async (
  email: string,
  password: string,
  userBackground: { software_background?: string; hardware_background?: string }
): Promise<{ error?: string; data?: { user: MockUser } }> => {
  // Simulate API call delay
  await new Promise(resolve => setTimeout(resolve, 500));

  // Mock successful registration
  mockAuthState = {
    ...mockAuthState,
    isAuthenticated: true,
    user: {
      id: '2',
      email,
      ...userBackground,
      created_at: new Date().toISOString(),
    },
  };
  return { data: { user: mockAuthState.user! } };
};

// Mock sign out function
export const mockSignOut = async (): Promise<void> => {
  // Simulate API call delay
  await new Promise(resolve => setTimeout(resolve, 300));

  mockAuthState = {
    isAuthenticated: false,
    user: null,
    isLoading: false,
  };
};

// Get current auth state
export const getMockAuthState = () => {
  return { ...mockAuthState };
};

// Mock useSession hook data
export const getMockSession = () => {
  return {
    data: mockAuthState.isAuthenticated ? { user: mockAuthState.user } : null,
    isLoading: mockAuthState.isLoading,
    error: null,
  };
};