const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000/api/v1';

export interface User {
  id: string;
  email: string;
  is_active: boolean;
  is_superuser: boolean;
  is_verified: boolean;
  software_background?: string;
  hardware_background?: string;
  created_at: string;
  last_login_at?: string;
}

// Helper to handle response errors
async function handleResponse(response: Response) {
  if (!response.ok) {
    let errorMessage = 'An error occurred';
    try {
      const data = await response.json();
      errorMessage = data.detail || data.message || errorMessage;
      if (Array.isArray(errorMessage)) {
          errorMessage = errorMessage.map(e => e.msg).join(', ');
      }
    } catch (e) {
      errorMessage = response.statusText;
    }
    throw new Error(errorMessage);
  }
  
  // For 204 No Content (like logout)
  if (response.status === 204) {
      return null;
  }
  
  return response.json();
}

export const signIn = async ({ email, password }: any) => {
  const formData = new URLSearchParams();
  formData.append('username', email); // FastAPI OAuth2PasswordRequestForm expects 'username'
  formData.append('password', password);

  const response = await fetch(`${API_URL}/auth/login`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/x-www-form-urlencoded',
    },
    body: formData,
  });

  return handleResponse(response);
};

export const signUp = async ({ email, password, software_background, hardware_background }: any) => {
  const response = await fetch(`${API_URL}/auth/register`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      email,
      password,
      is_active: true,
      is_superuser: false,
      is_verified: false,
      software_background,
      hardware_background,
    }),
  });

  return handleResponse(response);
};

export const signOut = async () => {
  const response = await fetch(`${API_URL}/auth/logout`, {
    method: 'POST',
  });
  return handleResponse(response);
};

export const getUser = async (): Promise<User | null> => {
  try {
    const response = await fetch(`${API_URL}/users/me`, {
        method: 'GET',
    });
    if (response.status === 401) {
        return null;
    }
    return handleResponse(response);
  } catch (error) {
    return null;
  }
};
