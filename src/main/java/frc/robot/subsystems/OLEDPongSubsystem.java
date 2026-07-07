<<<<<<< Updated upstream
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
=======
package frc.robot.subsystems;

>>>>>>> Stashed changes
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

<<<<<<< Updated upstream
=======
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

>>>>>>> Stashed changes
/**
 * OLED Pong Subsystem - Plays Pong with itself on a 128x64 SSD1306 OLED display
 * Connected to RoboRIO onboard I2C port (GND, 3.3V, SDA, SCL)
 */
public class OLEDPongSubsystem extends SubsystemBase {
  // SSD1306 Constants
  // Try 0x3D if display doesn't work with 0x3C
  private static final byte SSD1306_I2C_ADDRESS = 0x3C;
  private static final int SCREEN_WIDTH = 128;
  private static final int SCREEN_HEIGHT = 64;
  private static final int PAGES = SCREEN_HEIGHT / 8; // 8 pages of 8 pixels each

  // SSD1306 Commands
  private static final byte COMMAND_MODE = 0x00;
  private static final byte DATA_MODE = 0x40;

  // Pong Game State
  private double ballX = SCREEN_WIDTH / 2;
  private double ballY = SCREEN_HEIGHT / 2;
  private double ballVelX = 2.0;
  private double ballVelY = 1.5;
  private static final int BALL_SIZE = 2;

  private int leftPaddleY = SCREEN_HEIGHT / 2 - 8;
  private int rightPaddleY = SCREEN_HEIGHT / 2 - 8;
  private static final int PADDLE_HEIGHT = 16; // Bigger paddles = harder to miss
  private static final int PADDLE_WIDTH = 2;
  private static final int PADDLE_SPEED = 4; // Faster movement to always reach ball
  private static final int PADDLE_OFFSET = 3;

  // Paddle AI targeting
  private int leftPaddleTarget = SCREEN_HEIGHT / 2;
  private int rightPaddleTarget = SCREEN_HEIGHT / 2;
  private boolean ballWasOnLeft = true;
  private boolean ballWasOnRight = false;

  private int leftScore = 0;
  private int rightScore = 0;

  // Game state management
  private enum GameState { PLAYING, GAME_OVER }
  private GameState gameState = GameState.PLAYING;
  private long gameOverStartTime = 0;
  private static final long GAME_OVER_DURATION_MS = 1000;
  private boolean leftWon = false;

  private I2C i2c;
  private byte[] displayBuffer;

  // Background thread for OLED updates
  private ScheduledExecutorService oledThread;
  private static final int UPDATE_RATE_MS = 50; // Update display every 50ms (20 FPS)

  // Simple safety features
  private boolean oledEnabled = true;
  private boolean oledInitialized = false;

  // Simple 5x7 font for numbers and letters (minimal subset)
  private static final byte[][] FONT_5X7 = {
    // '0'
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    // '1'
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    // '2'
    {0x42, 0x61, 0x51, 0x49, 0x46},
    // '3'
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    // '4'
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    // '5'
    {0x27, 0x45, 0x45, 0x45, 0x39},
    // '6'
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    // '7'
    {0x01, 0x71, 0x09, 0x05, 0x03},
    // '8'
    {0x36, 0x49, 0x49, 0x49, 0x36},
    // '9'
    {0x06, 0x49, 0x49, 0x29, 0x1E}
  };

  // Letters for "GAME OVER" and "WIN"
  private static final byte[] LETTER_G = {0x3C, 0x42, 0x42, 0x52, 0x34};
  private static final byte[] LETTER_A = {0x7E, 0x09, 0x09, 0x09, 0x7E};
  private static final byte[] LETTER_M = {0x7F, 0x02, 0x04, 0x02, 0x7F};
  private static final byte[] LETTER_E = {0x7F, 0x49, 0x49, 0x49, 0x41};
  private static final byte[] LETTER_O = {0x3E, 0x41, 0x41, 0x41, 0x3E};
  private static final byte[] LETTER_V = {0x1F, 0x20, 0x40, 0x20, 0x1F};
  private static final byte[] LETTER_R = {0x7F, 0x09, 0x19, 0x29, 0x46};
  private static final byte[] LETTER_W = {0x7F, 0x20, 0x10, 0x20, 0x7F};
  private static final byte[] LETTER_I = {0x00, 0x41, 0x7F, 0x41, 0x00};
  private static final byte[] LETTER_N = {0x7F, 0x04, 0x08, 0x10, 0x7F};
  private static final byte[] LETTER_L = {0x7F, 0x40, 0x40, 0x40, 0x40};
  private static final byte[] LETTER_S = {0x46, 0x49, 0x49, 0x49, 0x31};
  private static final byte[] LETTER_DASH = {0x08, 0x08, 0x08, 0x08, 0x08};

  public OLEDPongSubsystem() {
    try {
      // Initialize I2C on onboard port
      i2c = new I2C(I2C.Port.kOnboard, SSD1306_I2C_ADDRESS);

      // Initialize display buffer (1024 bytes for 128x64 monochrome)
      displayBuffer = new byte[SCREEN_WIDTH * PAGES];

      // Initialize the OLED display
      initDisplay();
      oledInitialized = true;

      // Start background thread for Pong game
      oledThread = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r);
        t.setDaemon(true);
        t.setPriority(Thread.MIN_PRIORITY); // Lowest priority
        t.setName("OLED-Pong-Thread");
        return t;
      });

      // Schedule the game loop to run independently
      oledThread.scheduleAtFixedRate(this::runPongGame, 100, UPDATE_RATE_MS, TimeUnit.MILLISECONDS);

      DriverStation.reportWarning("OLED Pong started in background thread", false);
    } catch (Exception e) {
      DriverStation.reportError("OLED Pong initialization failed: " + e.getMessage(), false);
      oledEnabled = false;
    }
  }

  private void initDisplay() {
    // Basic SSD1306 initialization sequence
    sendCommand((byte)0xAE); // Display off
    sendCommand((byte)0xD5); // Set display clock divide ratio
    sendCommand((byte)0x80); // Suggested ratio
    sendCommand((byte)0xA8); // Set multiplex
    sendCommand((byte)0x3F); // 1/64 duty
    sendCommand((byte)0xD3); // Set display offset
    sendCommand((byte)0x00); // No offset
    sendCommand((byte)0x40); // Start line address
    sendCommand((byte)0x8D); // Charge pump
    sendCommand((byte)0x14); // Enable charge pump
    sendCommand((byte)0x20); // Memory mode
    sendCommand((byte)0x00); // Horizontal addressing mode
    sendCommand((byte)0xA0); // Segment re-map (horizontal flip for 180 rotation)
    sendCommand((byte)0xC0); // COM scan direction (vertical flip for 180 rotation)
    sendCommand((byte)0xDA); // Set COM pins
    sendCommand((byte)0x12); // Alternative COM pin config
    sendCommand((byte)0x81); // Set contrast
    sendCommand((byte)0xCF); // Contrast value
    sendCommand((byte)0xD9); // Set pre-charge period
    sendCommand((byte)0xF1); // Pre-charge value
    sendCommand((byte)0xDB); // Set VCOMH
    sendCommand((byte)0x40); // VCOMH value
    sendCommand((byte)0xA4); // Display all on resume
    sendCommand((byte)0xA6); // Normal display (not inverted)
    sendCommand((byte)0x2E); // Deactivate scroll
    sendCommand((byte)0xAF); // Display on
  }

  private void sendCommand(byte command) {
    if (!oledEnabled) return;
    byte[] data = {COMMAND_MODE, command};
    i2c.writeBulk(data);
  }

  private void sendCommand(byte command, byte value) {
    if (!oledEnabled) return;
    byte[] data = {COMMAND_MODE, command, value};
    i2c.writeBulk(data);
  }

  private void updateDisplay() {
    if (!oledEnabled) return;

    try {
      // Set column address
      sendCommand((byte)0x21); // Column address command
      sendCommand((byte)0x00); // Start column
      sendCommand((byte)0x7F); // End column (127)

      // Set page address
      sendCommand((byte)0x22); // Page address command
      sendCommand((byte)0x00); // Start page
      sendCommand((byte)0x07); // End page (7)

      // Send display buffer in chunks
      int chunkSize = 64;
      for (int i = 0; i < displayBuffer.length; i += chunkSize) {
        int remaining = Math.min(chunkSize, displayBuffer.length - i);
        byte[] chunk = new byte[remaining + 1];
        chunk[0] = DATA_MODE;
        System.arraycopy(displayBuffer, i, chunk, 1, remaining);
        i2c.writeBulk(chunk);
      }
    } catch (Exception e) {
      // Silently ignore errors - keep the game running
    }
  }

  private void clearBuffer() {
    for (int i = 0; i < displayBuffer.length; i++) {
      displayBuffer[i] = 0;
    }
  }

  private void setPixel(int x, int y, boolean on) {
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) return;

    int page = y / 8;
    int bit = y % 8;
    int index = page * SCREEN_WIDTH + x;

    if (on) {
      displayBuffer[index] |= (1 << bit);
    } else {
      displayBuffer[index] &= ~(1 << bit);
    }
  }

  private void drawRect(int x, int y, int width, int height) {
    drawRect(x, y, width, height, true);
  }

  private void drawRect(int x, int y, int width, int height, boolean on) {
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
        setPixel(x + i, y + j, on);
      }
    }
  }

  private void drawBall() {
    int ballXInt = (int)ballX;
    int ballYInt = (int)ballY;
    for (int i = 0; i < BALL_SIZE; i++) {
      for (int j = 0; j < BALL_SIZE; j++) {
        setPixel(ballXInt + i, ballYInt + j, true);
      }
    }
  }

  private void drawChar(byte[] charData, int x, int y) {
    for (int col = 0; col < charData.length; col++) {
      for (int row = 0; row < 7; row++) {
        if ((charData[col] & (1 << row)) != 0) {
          setPixel(x + col, y + row, true);
        }
      }
    }
  }

  private void drawText(String text, int x, int y) {
    int curX = x;
    for (char c : text.toCharArray()) {
      byte[] charData = null;
      if (c >= '0' && c <= '9') {
        charData = FONT_5X7[c - '0'];
      } else if (c == 'G') charData = LETTER_G;
      else if (c == 'A') charData = LETTER_A;
      else if (c == 'M') charData = LETTER_M;
      else if (c == 'E') charData = LETTER_E;
      else if (c == 'O') charData = LETTER_O;
      else if (c == 'V') charData = LETTER_V;
      else if (c == 'R') charData = LETTER_R;
      else if (c == 'W') charData = LETTER_W;
      else if (c == 'I') charData = LETTER_I;
      else if (c == 'N') charData = LETTER_N;
      else if (c == 'L') charData = LETTER_L;
      else if (c == 'S') charData = LETTER_S;
      else if (c == '-') charData = LETTER_DASH;
      else if (c == ' ') {
        curX += 4;
        continue;
      }

      if (charData != null) {
        drawChar(charData, curX, y);
        curX += 6;
      }
    }
  }

  private void drawGameOver() {
    // Clear middle area for text
    drawRect(20, 20, 88, 24, false); // Clear rectangle

    // Draw "GAME OVER" centered
    drawText("GAME OVER", 34, 22);

    // Draw score below (e.g., "3 - 5")
    String scoreText = leftScore + " - " + rightScore;
    int scoreX = 64 - (scoreText.length() * 3); // Rough centering
    drawText(scoreText, scoreX, 35);
  }

  // Center line removed for cleaner look

  private void updatePaddles() {
    boolean ballOnLeft = ballX < SCREEN_WIDTH / 2;
    boolean ballOnRight = !ballOnLeft;

    // Left paddle - nearly perfect prediction
    if (ballOnLeft) {
      // Calculate target only when ball first enters left side
      if (!ballWasOnLeft) {
        // Calculate EXACT position where ball will reach paddle
        double timeToReachPaddle = (ballX - PADDLE_OFFSET - PADDLE_WIDTH) / Math.abs(ballVelX);
        double predictedY = ballY + (ballVelY * timeToReachPaddle);

        // ALWAYS account for wall bounces perfectly
        while (predictedY < 0 || predictedY > SCREEN_HEIGHT - BALL_SIZE) {
          if (predictedY < 0) {
            predictedY = -predictedY;
          } else if (predictedY > SCREEN_HEIGHT - BALL_SIZE) {
            predictedY = 2 * (SCREEN_HEIGHT - BALL_SIZE) - predictedY;
          }
        }

        // Tiny random variation just for visual interest (±1 pixel max)
        predictedY += (Math.random() - 0.5) * 2;

        // Hit position varies slightly but always safely on paddle
        double hitOffset = (Math.random() - 0.5) * PADDLE_HEIGHT * 0.3; // Only middle 30% variation

        // Set target position - will always hit
        leftPaddleTarget = (int)(predictedY - PADDLE_HEIGHT / 2 + BALL_SIZE / 2 - hitOffset);
        leftPaddleTarget = Math.max(0, Math.min(SCREEN_HEIGHT - PADDLE_HEIGHT, leftPaddleTarget));
      }

      // Move smoothly but quickly enough to always reach target
      int distance = Math.abs(leftPaddleY - leftPaddleTarget);
      int moveSpeed = (distance > 10) ? PADDLE_SPEED + 1 : PADDLE_SPEED; // Speed up if far

      if (distance > 0) {
        if (leftPaddleY < leftPaddleTarget) {
          leftPaddleY = Math.min(leftPaddleY + moveSpeed, leftPaddleTarget);
        } else {
          leftPaddleY = Math.max(leftPaddleY - moveSpeed, leftPaddleTarget);
        }
      }
    }

    // Right paddle - nearly perfect prediction
    if (ballOnRight) {
      // Calculate target only when ball first enters right side
      if (!ballWasOnRight) {
        // Calculate EXACT position where ball will reach paddle
        double timeToReachPaddle = (SCREEN_WIDTH - PADDLE_OFFSET - PADDLE_WIDTH - BALL_SIZE - ballX) / Math.abs(ballVelX);
        double predictedY = ballY + (ballVelY * timeToReachPaddle);

        // ALWAYS account for wall bounces perfectly
        while (predictedY < 0 || predictedY > SCREEN_HEIGHT - BALL_SIZE) {
          if (predictedY < 0) {
            predictedY = -predictedY;
          } else if (predictedY > SCREEN_HEIGHT - BALL_SIZE) {
            predictedY = 2 * (SCREEN_HEIGHT - BALL_SIZE) - predictedY;
          }
        }

        // Tiny random variation just for visual interest (±1 pixel max)
        predictedY += (Math.random() - 0.5) * 2;

        // Hit position varies slightly but always safely on paddle
        double hitOffset = (Math.random() - 0.5) * PADDLE_HEIGHT * 0.3; // Only middle 30% variation

        // Set target position - will always hit
        rightPaddleTarget = (int)(predictedY - PADDLE_HEIGHT / 2 + BALL_SIZE / 2 - hitOffset);
        rightPaddleTarget = Math.max(0, Math.min(SCREEN_HEIGHT - PADDLE_HEIGHT, rightPaddleTarget));
      }

      // Move smoothly but quickly enough to always reach target
      int distance = Math.abs(rightPaddleY - rightPaddleTarget);
      int moveSpeed = (distance > 10) ? PADDLE_SPEED + 1 : PADDLE_SPEED; // Speed up if far

      if (distance > 0) {
        if (rightPaddleY < rightPaddleTarget) {
          rightPaddleY = Math.min(rightPaddleY + moveSpeed, rightPaddleTarget);
        } else {
          rightPaddleY = Math.max(rightPaddleY - moveSpeed, rightPaddleTarget);
        }
      }
    }

    // Remember ball position for next frame
    ballWasOnLeft = ballOnLeft;
    ballWasOnRight = ballOnRight;
  }

  private void updateBall() {
    // Move ball
    ballX += ballVelX;
    ballY += ballVelY;

    // Top/bottom wall collision
    if (ballY <= 0 || ballY >= SCREEN_HEIGHT - BALL_SIZE) {
      ballVelY = -ballVelY;
      ballY = Math.max(0, Math.min(SCREEN_HEIGHT - BALL_SIZE, ballY));
    }

    // Left paddle collision
    if (ballX <= PADDLE_OFFSET + PADDLE_WIDTH &&
        ballY + BALL_SIZE >= leftPaddleY &&
        ballY <= leftPaddleY + PADDLE_HEIGHT) {
      ballVelX = Math.abs(ballVelX);
      // Add some spin based on where it hits the paddle
      double hitPos = (ballY - leftPaddleY) / PADDLE_HEIGHT - 0.5;
      ballVelY += hitPos * 2;
    }

    // Right paddle collision
    if (ballX >= SCREEN_WIDTH - PADDLE_OFFSET - PADDLE_WIDTH - BALL_SIZE &&
        ballY + BALL_SIZE >= rightPaddleY &&
        ballY <= rightPaddleY + PADDLE_HEIGHT) {
      ballVelX = -Math.abs(ballVelX);
      // Add some spin based on where it hits the paddle
      double hitPos = (ballY - rightPaddleY) / PADDLE_HEIGHT - 0.5;
      ballVelY += hitPos * 2;
    }

    // Score and trigger game over if ball goes off screen
    if (ballX < 0) {
      rightScore++;
      leftWon = false;
      gameState = GameState.GAME_OVER;
      gameOverStartTime = System.currentTimeMillis();
      resetBall();
    } else if (ballX > SCREEN_WIDTH) {
      leftScore++;
      leftWon = true;
      gameState = GameState.GAME_OVER;
      gameOverStartTime = System.currentTimeMillis();
      resetBall();
    }

    // Keep scores reasonable - reset after either reaches 10
    if (leftScore >= 10 || rightScore >= 10) {
      leftScore = 0;
      rightScore = 0;
    }
  }

  private void resetBall() {
    ballX = SCREEN_WIDTH / 2;
    ballY = SCREEN_HEIGHT / 2;
    ballVelX = (Math.random() > 0.5 ? 2 : -2);
    ballVelY = (Math.random() - 0.5) * 4; // More vertical variation
  }

  private void runPongGame() {
    // Only run if OLED is enabled and initialized
    if (!oledEnabled || !oledInitialized) {
      return;
    }

    try {
      // Update game logic
      if (gameState == GameState.GAME_OVER &&
          System.currentTimeMillis() - gameOverStartTime > GAME_OVER_DURATION_MS) {
        gameState = GameState.PLAYING;
      }

      if (gameState == GameState.PLAYING) {
        updatePaddles();
        updateBall();
      }

      // Clear display buffer
      clearBuffer();

      if (gameState == GameState.PLAYING) {
        // Draw game elements
        drawRect(PADDLE_OFFSET, leftPaddleY, PADDLE_WIDTH, PADDLE_HEIGHT); // Left paddle
        drawRect(SCREEN_WIDTH - PADDLE_OFFSET - PADDLE_WIDTH, rightPaddleY, PADDLE_WIDTH, PADDLE_HEIGHT); // Right paddle
        drawBall();
      } else if (gameState == GameState.GAME_OVER) {
        // Show game over screen with scores
        drawRect(PADDLE_OFFSET, leftPaddleY, PADDLE_WIDTH, PADDLE_HEIGHT);
        drawRect(SCREEN_WIDTH - PADDLE_OFFSET - PADDLE_WIDTH, rightPaddleY, PADDLE_WIDTH, PADDLE_HEIGHT);
        drawGameOver();
      }

      // Send to display
      updateDisplay();
    } catch (Exception e) {
      // Silently continue - don't let exceptions stop the game
    }
  }

  @Override
  public void periodic() {
    // This runs in the main robot loop but does nothing
    // All OLED updates happen in the background thread
  }

  public void disableOLED() {
    oledEnabled = false;
    if (oledThread != null) {
      oledThread.shutdown();
    }
    DriverStation.reportWarning("OLED manually disabled", false);
  }

  public boolean isOLEDEnabled() {
    return oledEnabled;
  }

  // Test pattern to verify OLED is working
  public void drawTestPattern() {
    if (!oledEnabled || !oledInitialized) return;

    clearBuffer();
    // Draw a border
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      setPixel(x, 0, true);
      setPixel(x, SCREEN_HEIGHT - 1, true);
    }
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
      setPixel(0, y, true);
      setPixel(SCREEN_WIDTH - 1, y, true);
    }
    // Draw an X
    for (int i = 0; i < Math.min(SCREEN_WIDTH, SCREEN_HEIGHT); i++) {
      setPixel(i, i, true);
      setPixel(SCREEN_WIDTH - 1 - i, i, true);
    }
    updateDisplay();
    DriverStation.reportWarning("OLED test pattern drawn", false);
  }
}