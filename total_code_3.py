import cv2
import numpy as np
from datetime import datetime
################################
import serial
import struct
import time
import RPi.GPIO as GPIO
################################
fan_pin = 4
finger_cnt = [0,0,0]
GPIO.setmode(GPIO.BCM)  
GPIO.setup(fan_pin, GPIO.OUT)
GPIO.output(fan_pin, GPIO.LOW)
cv2.namedWindow('CAM', cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty('CAM',cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
################################

HUM_PIN = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(HUM_PIN, GPIO.OUT) 


# UART / USB Serial : 'dmesg | grep ttyUSB'
USB0 = '/dev/ttyUSB0'
UART = '/dev/ttyUSB0'

# USE PORT
SERIAL_PORT = UART

# Baud Rate
Speed = 9600

################################

class PMS7003(object):

    # PMS7003 protocol data (HEADER 2byte + 30byte)
    PMS_7003_PROTOCOL_SIZE = 32

    # PMS7003 data list
    HEADER_HIGH            = 0  # 0x42
    HEADER_LOW             = 1  # 0x4d
    FRAME_LENGTH           = 2  # 2x13+2(data+check bytes) 
    DUST_PM1_0_CF1         = 3  # PM1.0 concentration unit μ g/m3（CF=1，standard particle）
    DUST_PM2_5_CF1         = 4  # PM2.5 concentration unit μ g/m3（CF=1，standard particle）
    DUST_PM10_0_CF1        = 5  # PM10 concentration unit μ g/m3（CF=1，standard particle）
    DUST_PM1_0_ATM         = 6  # PM1.0 concentration unit μ g/m3（under atmospheric environment）
    DUST_PM2_5_ATM         = 7  # PM2.5 concentration unit μ g/m3（under atmospheric environment）
    DUST_PM10_0_ATM        = 8  # PM10 concentration unit μ g/m3  (under atmospheric environment) 
    DUST_AIR_0_3           = 9  # indicates the number of particles with diameter beyond 0.3 um in 0.1 L of air. 
    DUST_AIR_0_5           = 10 # indicates the number of particles with diameter beyond 0.5 um in 0.1 L of air. 
    DUST_AIR_1_0           = 11 # indicates the number of particles with diameter beyond 1.0 um in 0.1 L of air. 
    DUST_AIR_2_5           = 12 # indicates the number of particles with diameter beyond 2.5 um in 0.1 L of air. 
    DUST_AIR_5_0           = 13 # indicates the number of particles with diameter beyond 5.0 um in 0.1 L of air. 
    DUST_AIR_10_0          = 14 # indicates the number of particles with diameter beyond 10 um in 0.1 L of air. 
    RESERVEDF              = 15 # Data13 Reserved high 8 bits
    RESERVEDB              = 16 # Data13 Reserved low 8 bits
    CHECKSUM               = 17 # Checksum code


    # header check 
    def header_chk(self, buffer):

        if (buffer[self.HEADER_HIGH] == 66 and buffer[self.HEADER_LOW] == 77):
            return True

        else:
            return False

    # chksum value calculation
    def chksum_cal(self, buffer):

        buffer = buffer[0:self.PMS_7003_PROTOCOL_SIZE]

        # data unpack (Byte -> Tuple (30 x unsigned char <B> + unsigned short <H>))
        chksum_data = struct.unpack('!30BH', buffer)

        chksum = 0

        for i in range(30):
            chksum = chksum + chksum_data[i]

        return chksum

    # checksum check
    def chksum_chk(self, buffer):   
        
        chk_result = self.chksum_cal(buffer)
        
        chksum_buffer = buffer[30:self.PMS_7003_PROTOCOL_SIZE]
        chksum = struct.unpack('!H', chksum_buffer)
        
        if (chk_result == chksum[0]):
            return True

        else:
            return False

    # protocol size(small) check
    def protocol_size_chk(self, buffer):

        if(self.PMS_7003_PROTOCOL_SIZE <= len(buffer)):
            return True

        else:
            return False

    # protocol check
    def protocol_chk(self, buffer):
        
        if(self.protocol_size_chk(buffer)):
            
            if(self.header_chk(buffer)):
                
                if(self.chksum_chk(buffer)):
                    
                    return True
                else:
                    print("Chksum err")
            else:
                print("Header err")
        else:
            print("Protol err")

        return False 

    # unpack data 
    # <Tuple (13 x unsigned short <H> + 2 x unsigned char <B> + unsigned short <H>)>
    def unpack_data(self, buffer):
        
        buffer = buffer[0:self.PMS_7003_PROTOCOL_SIZE]

        # data unpack (Byte -> Tuple (13 x unsigned short <H> + 2 x unsigned char <B> + unsigned short <H>))
        data = struct.unpack('!2B13H2BH', buffer)

        return data


    def print_serial(self, buffer):
        
        chksum = self.chksum_cal(buffer)
        data = self.unpack_data(buffer)
        print ("0.3um in 0.1L of air : %s" % (data[self.DUST_AIR_0_3]))
        if data[self.DUST_AIR_0_3] >= 800:
            GPIO.output(HUM_PIN, GPIO.HIGH)
        else:
            GPIO.output(HUM_PIN, GPIO.LOW)
            
##############################################


# 원주 점 계산 함수
def calculate_circle_points(center, radius, num_points=100):
    angles = np.linspace(0, 2 * np.pi, num_points)
    points = []
    for angle in angles:
        x = int(center[0] + radius * np.cos(angle))
        y = int(center[1] + radius * np.sin(angle))
        points.append((x, y))
    return points


# Capture video from the webcam (0 is the default camera)
cap = cv2.VideoCapture(0)


# 웹캠 잘 열렸는지 체크
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()






###########################################
#serial setting 
ser = serial.Serial(SERIAL_PORT, Speed, timeout = 1)

dust = PMS7003()
###########################################



work_cnt = 0
fan_state = 0
hum_state = 0

while True:
    
    #########################################
    
    if work_cnt >= 150:         # 30fps 5sec, repeat 5sec
        ser.flushInput()
        buffer = ser.read(1024)

        if(dust.protocol_chk(buffer)):
        
            print("DATA read success")
        
            # print data
            dust.print_serial(buffer)
            
        else:

            print("DATA read fail...")
        
        work_cnt = 0
            
    ####################################
    
    if fan_state == 30:
        GPIO.output(fan_pin, GPIO.HIGH)
        fan_state -= 1
    elif fan_state > 0:
        fan_state -= 1
    elif fan_state <= 0:
        GPIO.output(fan_pin, GPIO.LOW)
    
    if hum_state == 300:
        GPIO.output(HUM_PIN, GPIO.HIGH)
        hum_state  -= 1
    elif hum_state >0:
        hum_state  -= 1
    elif hum_state <= 0:
        GPIO.output(HUM_PIN, GPIO.LOW)
    
    
    
    
    # 웹캠에서 프레임 읽기
    ret, frame = cap.read()
    work_cnt += 1
    
    # 프레임이 성공적으로 검색되지 않은 경우 루프 중단
    if not ret:
        print("Error: Could not read frame.")
        break
    
    height, width = frame.shape[:2]
    new_width = width // 2
    new_height = height // 2
    resized_frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)
    #frame = resized_frame
    
    # mask에 gaussianblur 적용
    frame_gaussian = cv2.GaussianBlur(resized_frame, (5, 5), 0)
    
    
    # red filter 생성
    red_filter = np.zeros_like(resized_frame)
    red_filter[:, :, 2] = 255  # Set the red channel to 255

    # red filter를 원래 프레임과 blend
    blended_frame = cv2.addWeighted(frame_gaussian, 0.85, red_filter, 0.15, 0)
    
    
    # 프레임을 HSV 색상 공간으로 변환
    hsv = cv2.cvtColor(blended_frame, cv2.COLOR_BGR2HSV)

    # HSV에서 피부색 범위 정의
    lower_skin = np.array([0, 80, 50], dtype=np.uint8)
    upper_skin = np.array([13, 255, 255], dtype=np.uint8)

    # 피부색으로부터 mask 만들기
    skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)

    # 형태학적 연산을 위한 구조 요소(kernel) 정의
    kernel = np.ones((5, 5), np.uint8)

    # noise제거 위해 mask에 closng operation 적용
    skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_CLOSE, kernel)

    # mask에 gaussianblur 적용
    skin_mask = cv2.GaussianBlur(skin_mask, (5, 5), 0)

    # Create an image with all pixels set to 0 (black)
    result = np.zeros_like(resized_frame)

    # Set the skin areas to 255 (white)
    result[skin_mask == 255] = 255

    # 결과를 grayscale로 convert
    gray_result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    # 연결된 구성요소 라벨링 적용
    num_labels, labels = cv2.connectedComponents(gray_result)

    # 배경제외 가장 큰 구성요소 찾기
    max_label = 1
    max_size = 0
    for label in range(1, num_labels):
        size = np.sum(labels == label)
        if size > max_size:
            max_size = size
            max_label = label

    # 가장 큰 구성요소만으로 최종 결과 이미지 생성
    final_result = np.zeros_like(gray_result)
    final_result[labels == max_label] = 255

    # 가장 큰 피부 영역 중심 찾기 위해 거리 변환 적용
    dist_transform = cv2.distanceTransform(final_result, cv2.DIST_L2, 5)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(dist_transform)
    center = max_loc  # Coordinates of the center pixel

    # 가장 큰 피부 부위의 중심에 원 그리기
    cv2.circle(resized_frame, center, 5, (0, 0, 255), -1)
    cv2.putText(resized_frame, f'Center: {center}', (center[0] , center[1] -20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    cv2.circle(final_result, center, 5, (0, 0, 255), -1)
    cv2.putText(final_result, f'Center: {center}', (center[0] , center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)


    # 감지 영역에서 총 픽셀 계산
    detected_area_size = np.sum(final_result == 255)
    total_pixels = final_result.shape[0] * final_result.shape[1]
    area_threshold = 0.12 * total_pixels
    
    
    
    if detected_area_size >= area_threshold:
        
        '''
        center_x = center[0]
        center_y = center[1]
        try:
            # Calculate the radius of the circle
            y_min, y_max = np.where(final_result[:,center_x] == 255)[0].min(), np.where(final_result[:,center_x] == 255)[0].max()
            #radius = int((y_max - y_min) *0.4)
            x_min, x_max = np.where(final_result[center_y] == 255)[0].min(), np.where(final_result[center_y] == 255)[0].max()
            x_diff = x_max-x_min
            y_diff = y_max-y_min
            if x_diff < y_diff:
                radius = int(x_diff *0.65)
            else :
                radius = int(y_diff *0.65)
        except:
            radius = 0
        '''
        radius = int(new_height/3)
        
        '''
        # Calculate the radius of the circle
        y_min, y_max = np.where(final_result == 255)[0].min(), np.where(final_result == 255)[0].max()
        #radius = int((y_max - y_min) *0.4)
        x_min, x_max = np.where(final_result == 255)[1].min(), np.where(final_result == 255)[1].max()
        x_diff = x_max-x_min
        y_diff = y_max-y_min
        if x_diff < y_diff:
            radius = int(x_diff *0.55)
        else:
            radius = int(y_diff *0.55)

        '''
        # 점들의 원주 계산
        circle_points = calculate_circle_points(center, radius)

                
        if center[1] > int(new_height/4) and center[1] < int(new_height*3/4) :
            if center[0] > int(new_width/4) and center[0] < int(new_width*3/4) :
                
                # 지점의 grayscale 값 추출
                grayscale_values = []
                for point in circle_points:
                    if 0 <= point[1] < gray_result.shape[0] and 0 <= point[0] < gray_result.shape[1]:
                        grayscale_values.append(gray_result[point[1], point[0]])
                    else:
                        grayscale_values.append(0)
                        
                        
                transitions = -1
                for i in range(1, len(grayscale_values)):
                    if grayscale_values[i-1] == 0 and grayscale_values[i] == 255:
                        transitions += 1
                        
                cv2.putText(resized_frame, f'{transitions}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f'{transitions}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)



                if transitions == 1:
                    finger_cnt[0] += 1
                    finger_cnt[1] -= 3
                    finger_cnt[2] -= 3
                elif transitions == 2:
                    finger_cnt[1] += 1
                    finger_cnt[0] -= 3
                    finger_cnt[2] -= 3
                elif transitions == 3:
                    finger_cnt[2] += 1
                    finger_cnt[0] -= 3
                    finger_cnt[1] -= 3
                else:
                    finger_cnt[0] -= 3
                    finger_cnt[1] -= 3
                    finger_cnt[2] -= 3
                    
                if finger_cnt[0] < 0:
                    finger_cnt[0] = 0
                if finger_cnt[1] < 0:
                    finger_cnt[1] = 0
                if finger_cnt[2] < 0:
                    finger_cnt[2] = 0

                if finger_cnt[0] > 10:
                    print("1 finger !")
                    cv2.putText(frame, '1', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    fan_state = 30
                    finger_cnt[0] = 0
                    finger_cnt[1] = 0
                    finger_cnt[2] = 0
                elif finger_cnt[1] > 10:
                    print("2 finger !")
                    hum_state = 300
                    cv2.putText(frame, '2', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    finger_cnt[0] = 0
                    finger_cnt[1] = 0
                    finger_cnt[2] = 0
                elif finger_cnt[2] > 10:
                    print("3 finger !")
                    
                    cv2.putText(frame, '3', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    finger_cnt[0] = 0
                    finger_cnt[1] = 0
                    finger_cnt[2] = 0
                    # 현재 시간 get
                    current_time = datetime.now()
                    # 현재시간을 string으로 포맷
                    time_str = current_time.strftime("%Y%m%d_%H%M%S")
                    # 현재시간 파일이름 만들기
                    
                    filename = f"saved_frame_{time_str}.jpg"
                    
                    # 현재 프레임 저장
                    cv2.imwrite(filename, frame)
        
            
            
        # 반지름으로 중심에서 원 그리기
        cv2.circle(resized_frame, center, radius, (0, 0, 255), 2)
        cv2.circle(final_result, center, radius, (0, 0, 255), 2)

        
        pass
    else:
        pass
    cv2.imshow('CAM', frame)
        
    
    # Wait for 1 ms and break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
ser.close()
# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
