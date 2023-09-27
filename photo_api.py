from __future__ import print_function
import os, time, sys, datetime
from googleapiclient.discovery import build
from httplib2 import Http
from oauth2client import file, client, tools
import wget

# If modifying these scopes, delete the file token.json.
SCOPES = ["https://www.googleapis.com/auth/photoslibrary",
          "https://www.googleapis.com/auth/photoslibrary.readonly",
          "https://www.googleapis.com/auth/photoslibrary.readonly.appcreateddata"]

def main():
    #OAuth2 authentication process
    print("OAuth2 authentication process...")

    store = file.Storage("token.json")
    creds = store.get()
    if not creds or creds.invalid:
        flow = client.flow_from_clientsecrets("client_secret_240566316207-lcq43h4vipivrg519crr66pjeqgrbreo.apps.googleusercontent.com.json", SCOPES)
        creds = tools.run_flow(flow, store)
    service = build("photoslibrary", "v1", http=creds.authorize(Http()))

main()

