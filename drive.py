import sys
import os

from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive

name=sys.argv[1]


gauth = GoogleAuth()
gauth.LoadCredentialsFile("mycreds.txt")
if gauth.credentials is None:
    # Authenticate if they're not there
    gauth.LocalWebserverAuth()
elif gauth.access_token_expired:
    # Refresh them if expired
    gauth.Refresh()
else:
    # Initialize the saved creds
    gauth.Authorize()
# Save the current credentials to a file
gauth.SaveCredentialsFile("mycreds.txt")
drive=GoogleDrive(gauth)

photo=drive.CreateFile()
photo.SetContentFile(name)
photo.Upload()
os.remove(name)

