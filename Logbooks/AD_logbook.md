# Logbook - Antoine

## 19/05/2025

First day of project. Built a high-level diagram covering the different software aspects of the project and how they link to one another, such as the UI, the backend website server, TCP server, microcontroller, AWS...
Also looked at past reports, thoroughly read through the instructions and documents and looked at different pathfinding algorithms for how to deal with the autonomous navigation

## 20/05/2025

Second day. Started researching how to build a website using JavaScript, HTML & CSS. At first I want to build a simple website with nothing much on it which just has the capacity to connect to the Raspberry Pi, then I want to add functionalities like seeing battery usage, time, being able to control the robot...
Thinking about what pathfinding algorithm would be optimal - A*, Dijsktra or something else perhaps?

Created a test website to see if it can communicate with the Raspberry Pi:

![image](https://github.com/user-attachments/assets/c4276dd6-73ed-4b51-abad-90e6941cfa66)

only option is click me:

![image](https://github.com/user-attachments/assets/1fc2818f-bbc8-41ce-8aca-879aa63aaf54)

We then created a basic server.js, git cloned the repo into the raspberry pi, downloaded Node.js and Express onto it, checked the IP via hostname -I and was able to successfully run the website onto the raspberry pi (both PC and raspberry pi connected to hotspot)

![image](https://github.com/user-attachments/assets/6f8fdb4e-5278-4caa-aeb9-7a2c6cda4053)

Added a flash LED button to the website: we can click it as many times as we wish and every time we do, we flash the LED

![image](https://github.com/user-attachments/assets/b0d10b7d-19a0-4b2b-8ba8-139483545fa4)

## 21/05/2025

In order to connect the Raspberry pi and make it run the website:
  - Download Python, Node, Express onto the Raspberry Pi (already done in our case)
  - No need to git clone this repo as it is already done, instead
    "cd AenerShark" => "git pull" => "cd AenerShark_Website"
  - type "hostname -I" and you will get an IP of the form: 172.20.10.2
  - type "node server.js" and you will get a message of the form: Server running at http://localhost:5000
  - on your PC, type "http://172.20.10.2:5000" (or replace 172.20.10.2 by the IP you were given and 5000 by the port number you were given after typing node server.js"
  - make sure the Raspberry Pi / laptop are connected to the same WiFi/mobile hotspot

Updated the website to have two tabs: PID values & Testing

![image](https://github.com/user-attachments/assets/3a9b5ea3-0a4b-4858-a5f0-938df428e5cf)

When connected to the Raspberry Pi, the website will give the user the ability to type in integer values for the different PID parameters and will have a table which stores these past inputs. 

## 22/05/2025

The draft website currently implemented is mostly comprised of index.html (the website is opened by simply clicking on index.html), as well as server.js, script.js and styles.css. 

Researched MQTT, why use it over HTTP, and how to implement it. In our case we will use the Paho library in Python to implement MQTT. 
We needed a broker to establish MQTT connections, such as a free-tier EC2 t2.micro running Mosquitto or HiveMQ. 
We measure the average RTT for a large number of messages in order to measure the performance of MQTT. 

MQTT (Message Queuing Telemetry Transport) is a publish-subscribe application layer protocol designed for constrained devices and low-bandwidth, high-latency, or unreliable networks. It is optimized for low power, low memory, and low network usage environments. It uses TCP/IP as its transport protocol. Nonetheless, MQTT can sometimes suffer from memory leaks, connection failures or message delivery problems. We can try and deal with these issues by using well-established and maintained libraries such as Paho, monitor memory usage, keep queues short, enable session persistence, use keep-alive pings, and enable automatic reconnect on the client-side (reconnect_delay_set() in Paho). HiveMQ allows us to do load balancing and we should try and minimise message size.

| Feature                | MQTT                                | HTTP                                 | WebSocket                           |
|------------------------|-------------------------------------|--------------------------------------|-------------------------------------|
| **Communication Type** | Publish/Subscribe                   | Request/Response                     | Full-duplex, Bidirectional          |
| **Connection**         | Persistent (TCP)                    | Stateless (new TCP for each request) | Persistent (over TCP)               |
| **Latency**            | Low                                 | Moderate                             | Low                                 |
| **Efficiency**         | High (small packets, low overhead)  | Low (larger headers, repeated setup) | Moderate                            |
| **Security**           | SSL/TLS                             | SSL/TLS                              | SSL/TLS                             |
| **Resource Usage**     | Very Low                            | High                                 | Moderate                            |
| **Requirement**        | Broker (e.g., Mosquitto)            | Web Server                           | WebSocket Server                    |
| **QoS Support**        | Yes                                 | No                                   | No                                  |
| **Built-in Ack**       | Yes                                 | Yes (via status codes)               | Limited (application-defined)       |
| **Mobile Support**     | Good                                | Good                                 | Good                                |


We then needed to update our web backend so we made a comparison of the relevant framework and APIs:

| **Feature**            | **FastAPI**                           | **Python REST API (Generic)**       | **Node.js**                         | **Flask**                            |
|------------------------|---------------------------------------|-------------------------------------|-------------------------------------|--------------------------------------|
| **Type**               | Web framework (Python)                | API toolkit/framework               | Runtime + frameworks (e.g. Express)| Web framework (Python)               |
| **Language**           | Python                                | Python                              | JavaScript / TypeScript             | Python                               |
| **Performance**        | Very high                             | Moderate                            | High                                | Moderate                             |
| **Ease of Use**        | High                                  | Moderate                            | Moderate                            | Very high                            |
| **Community Support**  | Growing rapidly                       | Mature (Django/DRF very popular)    | Massive                             | Mature                               |
| **Built-in Routing**   | Yes                                   | Yes (DRF / Flask-RESTful, etc.)     | Yes (Express, Fastify)              | Yes                                  |
| **Scalability**        | Good                                  | Moderate                            | Excellent                           | Limited 


I also got to do some research on different pathfinding algorithms:

| Algorithm         | Handles Weights? | Uses Heuristic? | Optimal?                        | Notes                               |
| ----------------- | ---------------- | --------------- | ------------------------------- | ----------------------------------- |
| BFS               | No               | No              | Yes (unweighted)                | Good for unweighted grids           |
| DFS               | No               | No              | No                              | Can go deep but take bad paths      |
| Dijkstra          | Yes              | No              | Yes                             | Slower than A\* with heuristics     |
| Greedy Best-First | Yes              | Yes             | No                              | Fast, not optimal                   |
| A\*               | Yes              | Yes             | Yes (with admissible heuristic) | Best general-purpose algorithm      |
| Bidirectional     | Depends          | Optional        | Depends                         | Faster in certain structured graphs |
| Floyd-Warshall    | Yes              | No              | Yes                             | All-pairs, slow                     |

This led me to the conclusion that A* is the best pathfiding algorithm, especially in the context of a robot which has the ability to find paths and follow people.

## 23/05/2025

Finally managed to create the UI for an arrow pad with a stop button in the middle. 
I implemented a click and hold logic. 

![image](https://github.com/user-attachments/assets/b5d8ff19-0729-44ed-8c55-d3886cb05dff)

I then added the Aenertia logo to the website so it looks like this:

![image](https://github.com/user-attachments/assets/2a834e82-afbf-470f-b8cd-7558645d683e)

The server is implemented using FastAPI, and the link to access the website I am using is set as http://localhost:8000/pad

The next step will be to control the robot not only by clicking buttons on the website but also by using the arrows on my keyboard, which would lead to a slight animation of the arrows on the website.
I will also try and make the website look better by enhancing some of the effects and adding some color.

Made some changes to the website:

![image](https://github.com/user-attachments/assets/237f3a62-89c0-432c-9bf3-cf2ca41f08ed)

I tried to make the control pad neater, and now the robot can be controlled not only using the arrow keys on the website but also using the keyboard arrows.
In the top right corner I also added a battery feature (the battery of my computer - this is a draft, in the future this will be the battery of the robot) as well as the time.

I would like for this website to have 3 separate tabs (PID control, flash LED and control robot pad) in the future to make the website more neat and organized.

For the camera access via the Raspberry Pi, we must unpower the Raspberry Pi, connect the camera and then power it up again, otherwise the camera will not be detected. 

## 27/05/2025

Implemented tabs in the web page with a hover effect when you roll your mouse pointer over the tabs
The new URL I use is http://localhost:8000/dashboard

Arrow pad tab:
(same as previously, with battery and time - can use arrows on the keyboard too)
![image](https://github.com/user-attachments/assets/e0af73ff-74c9-4795-8e86-04f74f4f9f0e)

Flash LED tab:
![image](https://github.com/user-attachments/assets/fb7fd93d-3177-4a64-ba3c-018e80da2b5e)

Control values tab:
![image](https://github.com/user-attachments/assets/12b6dd3e-7cbd-42fe-a55a-45c1d6e55f03)

I had to change the position of the server.py code displaying time and battery so that it could appear on all 3 tabs

I then added 3 different mode types: manual, autonomous and test:

![image](https://github.com/user-attachments/assets/986e52c5-8fb0-4faa-9284-1fbda5848de9)

I updated the autonomous tab to have a saved key locations button, which displays all locations which have been inputted in the "assign" box: clicking on a given location listed on the website sets it as the key location for the robot.

![image](https://github.com/user-attachments/assets/c2ea6f92-8c2c-486f-b738-8ff6c5b2ddc9)

To access the website:
  - git pull
  - head to AenerShark\tests\communication_test
  - run uvicorn server:app --host 0.0.0.0 --port 8000
  - head to http://localhost:8000/dashboard

As of now, our current system can be summarised to:

![image](https://github.com/user-attachments/assets/ccdfc337-3624-4e26-b427-b60b2ec572f9)

This allows for a scalable project which can have its values loaded onto an AWS account (via DynamoDB & S3)

## 28/05/2025

Changed the interface to show MQTT status and added the ability to assign to key locations on every tab

![image](https://github.com/user-attachments/assets/a24c22e2-754c-43a4-9f8c-0709c0c26ebb)

I then reformatted the website so that the show key locations button is next to the assign key location button:

![image](https://github.com/user-attachments/assets/e1329ded-95c1-4676-809c-ba6502f876b3)

During the UI-backend integration, we decided that FastAPI was no longer necessary:

| **Functionality**           | **FastAPI**     | **Now **                                    |
| --------------------------- | ----------------------- | ---------------------------------------------- |
| Serving frontend            | FastAPI routes          | **Static hosting** (e.g., S3, or local server) |
| Robot → backend data flow   | API endpoints           | **MQTT**                                       |
| Database writes             | API + FastAPI + boto3   | **AWS Lambda** triggered by S3 / MQTT          |
| File uploads                | API handling in FastAPI | **S3 direct upload + Lambda post-processing**  |
| PID / telemetry interaction | API endpoints           | **MQTT topics** (e.g., `robot/pid/inner`)      |

We now have a server-less, event-driven architecture, as we realised we didn't need a formal framework or backend. 

I will start working on implementing the database from here on out.

## 29/05/2025

We decided to use DynamoDB as a NoSQL database for the course of our project 

| Factor             | DynamoDB     | MongoDB      | PostgreSQL      | Firebase        | Redis           | SQLite |
| ------------------ | ------------ | ------------ | --------------- | --------------- | --------------- | ------ |
| Serverless         |  Yes        | Via Atlas |  RDS setup    | Yes           | Yes           | No     |
| Local Dev Friendly | Emulator  |  Yes        |  Yes           |  Emulator      |  Local Redis   | Yes  |
| MQTT Integration   |  Via Lambda |  Via SDKs   |  Manual logic |  Manual logic |  Pub/Sub only | No      |
| AWS Integration    |  Native     |  External  | (RDS/Aurora)  | No              |  (Elasticache) | No      |

My goals for now are to implement a functional (albeit simple for now) database and to use React for the website.

## 02/06/2025

Did the interim presentation this morning after some prep work.
Started filling up the server folder, especially in the database subfolder. 

[Raspberry Pi]

    |
    
    | MQTT publish (sensor data)
    
    v
    
[AWS IoT Core]

    |
    
    | IoT Rule (Trigger)
    
    v
    
[Lambda Function]

    |
    
    | put_item(...)
    
    v
    
[Amazon DynamoDB]

Above is the structure I will use tomorrow to connect the Raspberry Pi in real time with the DynamoDB

## 05/06/2025

Successfully connected the group repo to S3 by using a test file called test_s3_upload.py, linked to the main s3_upload.py file. 

![image](https://github.com/user-attachments/assets/9118ad09-95ce-4e50-bb58-8fca672f7215)

After granting my user and bucket the appropriate permissions (and naming the bucket correctly, as it needs a unique global name), I can see the test file appearing in the aenershark-uploads bucket in AWS S3. 

![image](https://github.com/user-attachments/assets/f377ed1c-4bc0-4e40-aff2-1eca85bd7444)


Within the bucket, we have

![image](https://github.com/user-attachments/assets/88c65729-ef32-475f-9df0-f6db87630c07)

and within the folder, we have

![image](https://github.com/user-attachments/assets/3459819e-a354-41e1-a1fc-f7b7a16cce53)

which we can then open to get:

![image](https://github.com/user-attachments/assets/102a4446-097b-480f-86e7-c064bf072844)

We were able to implement this via the s3_upload.py file

```py
s3 = session.client("s3")
BUCKET_NAME = "aenershark-uploads"  

def upload_file_to_s3(local_path, bucket_name=BUCKET_NAME, s3_key=None):
    import os
    from botocore.exceptions import ClientError

    if not os.path.isfile(local_path):
        print(f"File does not exist: {local_path}")
        return None

    if not s3_key:
        s3_key = os.path.basename(local_path)

    try:
        s3.upload_file(local_path, bucket_name, s3_key)
        print(f"Uploaded {local_path} to s3://{bucket_name}/{s3_key}")
        return f"s3://{bucket_name}/{s3_key}"
    except ClientError as e:
        print(f"Upload failed: {e}")
        return None
```

which is imported by the test_s3_upload.py file

```py
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from server.s3bucket.s3_upload import upload_file_to_s3

file_to_upload = os.path.join(os.path.dirname(__file__), "my_test.txt")
s3_key = "uploads/my_test.txt" 

print("📤 Uploading to S3...")
result = upload_file_to_s3(file_to_upload, s3_key=s3_key)

if result:
    print(f"✅ Upload complete! File is at: {result}")
else:
    print("❌ Upload failed.")
```

I also managed to successfully add and test a Lambda function to my AWS:

![image](https://github.com/user-attachments/assets/06dac3bb-54cf-4029-8d89-607931f08ced)

I set the handler to store_metadata.lambda_handler.

After zipping some test data and deploying it to the Lambda function I created, I created two test functions in the event editor:

1) BatteryLowTest

![image](https://github.com/user-attachments/assets/d62c720d-4634-4169-a01c-6eaaeecb24a2)

2) BatteryOkayTest

![image](https://github.com/user-attachments/assets/894252ce-2749-4c69-99a1-e7b0cede0151)

This shows that with a battery level under 10%, we get a warning message, but above 10% we don't, thus the lambda function is able to run properly. 

I then added a trigger aenershark-uploads which makes storeMetaDataLambda to automatically trigger when a new file is added to the S3 bucket

![image](https://github.com/user-attachments/assets/60bd5a20-7f70-4bde-b4d5-f305e9c1ef02)

Inside the lambda folder on the repo, I added store_metadata.py:

```py
import json

def lambda_handler(event, context):
    print("📦 Event received:", json.dumps(event, indent=2))

    if "Records" in event and event["Records"][0]["eventSource"] == "aws:s3":
        record = event["Records"][0]
        bucket = record["s3"]["bucket"]["name"]
        key = record["s3"]["object"]["key"]
        s3_path = f"s3://{bucket}/{key}"
        print(f"🚀 Triggered by upload: {s3_path}")
        return {
            "statusCode": 200,
            "body": json.dumps(f"Processed upload: {s3_path}")
        }

    try:
        battery_level = event["metadata"]["battery_level"]
        if battery_level < 10:
            return {
                "statusCode": 200,
                "body": json.dumps(f"⚠️ WARNING: Battery low ({battery_level}%)")
            }
        else:
            return {
                "statusCode": 200,
                "body": json.dumps("✅ Battery level OK")
            }
    except (KeyError, TypeError):
        return {
            "statusCode": 400,
            "body": json.dumps("Invalid input format.")
        }
```

and inside CloudWatch, we can see that (after editing the code in the lambda function) when we re-run python server/s3bucket/test_s3_upload.py in our terminal, we see that the lambda trigger is successfully activated:

![image](https://github.com/user-attachments/assets/2e31538e-1453-49da-9aa9-0331ff120378)

## 10/06/2025

| **Feature**           | **Node.js**<br>| **Flask**<br> | **FastAPI**<br> | **Static Website + MQTT**<br> |
| --------------------- | ------------------------------ | --------------------------------- | ------------------------------ | ----------------------------------------------- |
| **Type**              | Runtime + web framework        | Python web framework              | Python web framework   | Frontend + real-time messaging                  |
| **Language**          | JavaScript       | Python                            | Python                         | HTML / JS                                |
| **Performance**       | High                           | Moderate                          | Very high                      | Dependent on MQTT backend / browser             |
| **Ease of Use**       | Moderate                       | Very high                         | High                           | Very high                                       |
| **Built-in Routing**  | Yes (via Express, etc.)        | Yes                               | Yes                            | No (static routing + JS logic)                  |
| **Scalability**       | Excellent                      | Limited                           | Good                           | Good (serverless-ready)                         |



