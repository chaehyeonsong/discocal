FROM jekyll/jekyll:pages

RUN apk update && apk add --no-cache \
    build-base \
    libssl1.1 \
    make \
    bash \
    && rm -rf /var/cache/apk/*

WORKDIR /srv/jekyll

COPY Gemfile* ./

RUN bundle install
CMD ["bundle", "exec", "jekyll", "serve", "--host", "0.0.0.0"]