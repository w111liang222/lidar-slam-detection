from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn
import sys, os, time, html, zipfile

PORT = int(sys.argv[1])

def fbytes(B):
   'Return the given bytes as a human friendly KB, MB, GB, or TB string'
   B = float(B)
   KB = float(1024)
   MB = float(KB ** 2) # 1,048,576
   GB = float(KB ** 3) # 1,073,741,824
   TB = float(KB ** 4) # 1,099,511,627,776

   if B < KB:
      return '{0} {1}'.format(B,'Bytes' if 0 == B > 1 else 'Byte')
   elif KB <= B < MB:
      return '{0:.2f} KB'.format(B/KB)
   elif MB <= B < GB:
      return '{0:.2f} MB'.format(B/MB)
   elif GB <= B < TB:
      return '{0:.2f} GB'.format(B/GB)
   elif TB <= B:
      return '{0:.2f} TB'.format(B/TB)

def send_head(self):
    """Common code for GET and HEAD commands.

    This sends the response code and MIME headers.

    Return value is either a file object (which has to be copied
    to the outputfile by the caller unless the command was HEAD,
    and must be closed by the caller under all circumstances), or
    None, in which case the caller has nothing further to do.

    """
    path = self.translate_path(self.path)
    f = None

    if self.path.endswith('?download'):
        zip_file = ".zip"
        self.path = self.path.replace("?download","")

        zip = zipfile.ZipFile(zip_file, 'w')
        for root, dirs, files in os.walk(path):
            for file in files:
                if os.path.join(root, file) != os.path.join(root, zip_file):
                    zip.write(os.path.join(root, file), os.path.join(root, file).replace(path, ""))
        zip.close()
        path = self.translate_path(zip_file)

    elif os.path.isdir(path):
        if not self.path.endswith('/'):
            # redirect browser - doing basically what apache does
            self.send_response(301)
            self.send_header("Location", self.path + "/")
            self.end_headers()
            return None
        else:
            for index in "index.html", "index.htm":
                index = os.path.join(path, index)
                if os.path.exists(index):
                    path = index
                    break
            else:
                return self.list_directory(path)
    ctype = self.guess_type(path)
    try:
        # Always read in binary mode. Opening files in text mode may cause
        # newline translations, making the actual size of the content
        # transmitted *less* than the content-length!
        f = open(path, 'rb')
    except IOError:
        self.send_error(404, "File not found")
        return None
    self.send_response(200)
    self.send_header("Content-type", ctype)
    fs = os.fstat(f.fileno())
    self.send_header("Content-Length", str(fs[6]))
    self.send_header("Last-Modified", self.date_time_string(fs.st_mtime))
    self.end_headers()
    return f

def list_directory(self, path):
    from io import BytesIO
    import html, urllib

    """Helper to produce a directory listing (absent index.html).

    Return value is either a file object, or None (indicating an
    error).  In either case, the headers are sent, making the
    interface the same as for send_head().

    """
    try:
        list = os.listdir(path)
    except os.error:
        self.send_error(404, "No permission to list directory")
        return None
    list.sort(key=lambda a: a.lower())
    f = BytesIO()
    displaypath = html.escape(urllib.parse.unquote(self.path))
    f.write(b'<!DOCTYPE html PUBLIC "-//W3C//DTD HTML 3.2 Final//EN">')
    f.write(b"<html>\n<title> TSARI </title>\n")
    f.write(("<body>\n<h1>Directory listing for %s</h1>\n" % displaypath).encode())
    # f.write(("<a href='%s'>%s</a>\n" % (self.path+"?download",'Download Current Directory')).encode())
    f.write(b"<hr>\n")
    f.write(b'<table>\n')
    for name in list:
        fullname = os.path.join(path, name)
        displayname = linkname = name
        fsize = fbytes(os.path.getsize(fullname))
        created_date = time.ctime(os.path.getctime(fullname))
        # Append / for directories or @ for symbolic links
        if os.path.isdir(fullname):
            displayname = name + "/"
            linkname = name + "/"
        if os.path.islink(fullname):
            displayname = name + "@"
            # Note: a link to a directory displays with @ and links with /
        f.write(('<tr><td><a href="%s">%s</a><td>         </td><td> </td><td style="text-align:right; font-weight: bold; color:#FF0000">%s</td><td style="text-align:right; font-weight: bold;">%s</td></tr>\n'
                    % ( urllib.parse.quote(linkname), html.escape(displayname) , fsize , created_date )).encode())
    f.write(b"</table><hr>\n</body>\n</html>\n")
    length = f.tell()
    f.seek(0)
    self.send_response(200)
    encoding = sys.getfilesystemencoding()
    self.send_header("Content-type", "text/html; charset=%s" % encoding)
    self.send_header("Content-Length", str(length))
    self.end_headers()
    return f

Handler = SimpleHTTPRequestHandler
Handler.send_head = send_head
Handler.list_directory = list_directory

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

if __name__ == '__main__':
    if sys.argv[2:]:
        os.chdir(sys.argv[2])
    server = ThreadedHTTPServer(('0.0.0.0', PORT), Handler)
    server.serve_forever()