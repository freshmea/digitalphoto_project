import pickle
import os.path
import datetime
from googleapiclient.discovery import build
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
import wget
import random

# If modifying these scopes, delete the file token.pickle.
SCOPES = ['https://www.googleapis.com/auth/photoslibrary.readonly']

creds = None
# The file token.pickle stores the user's access and refresh tokens, and is
# created automatically when the authorization flow completes for the first
# time.
if os.path.exists('token.pickle'):
    with open('token.pickle', 'rb') as token:
        creds = pickle.load(token)
# If there are no (valid) credentials available, let the user log in.
if not creds or not creds.valid:
    if creds and creds.expired and creds.refresh_token:
        creds.refresh(Request())
    else:
        flow = InstalledAppFlow.from_client_secrets_file(
            'credentials.json', SCOPES)
        creds = flow.run_local_server(port=0)
    # Save the credentials for the next run
    with open('token.pickle', 'wb') as token:
        pickle.dump(creds, token)

service = build('photoslibrary', 'v1', credentials=creds, static_discovery=False)

#Search image files
print("Search image files...")
dtToday = datetime.date.today()
dt7DaysAgo = dtToday - datetime.timedelta(days=7)

results = service.mediaItems().search(
    body={
        "filters":{
            "dateFilter":{
                "ranges":[
                    {"startDate":{"year":dt7DaysAgo.year,"month":dt7DaysAgo.month,"day":dt7DaysAgo.day},
                        "endDate":{"year":dtToday.year,"month":dtToday.month,"day":dtToday.day}
                        }
                ]
            },
            "mediaTypeFilter":{
                "mediaTypes":["PHOTO"]
            }
        }
    }
).execute()

items = results.get("mediaItems", [])
if not items:
    print("No media found.")
    quit()
print(items)

#Download & conver image files
print("Dowload & Convert image files...")
# for item in items:
item = random.choice(items)
filename = item["filename"]
baseUrl = item["baseUrl"]

print(f"\nfilename:{filename}, {baseUrl}")

# check duplication & download
mypath = '/home/jetson/Pictures/sonii'
print(type(filename), filename)
file_full_path = os.path.join(mypath,filename)
if not os.path.isfile(file_full_path):
    wget.download(baseUrl, file_full_path)
    # command = f"convert {file_full_path} -background white -gravity center -colorspace Gray -resize x176 -extent 264x176 -rotate '90>' images/{1}".format(filename, bmpname)
    # os.system(command)

# delete all jpg files
print("Deleting all jpg files...")
command = "rm -rf images/*.jpg"
os.system(command)