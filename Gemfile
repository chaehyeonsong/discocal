source "https://rubygems.org"

# Hello! This is where you manage which Jekyll version is used to run.
# When you want to use a different version, change it below, save the
# file and run `bundle install`. Run Jekyll with `bundle exec`, like so:
#
#     bundle exec jekyll serve
#
# This will help ensure the proper Jekyll version is running.
# Happy Jekylling!
# gem "jekyll", "~> 3.10.0"

# This is the default theme for new Jekyll sites. You may change this to anything you like.
gem "minima", "~> 2.0"

# If you want to use GitHub Pages, remove the "gem "jekyll"" above and
# uncomment the line below. To upgrade, run `bundle update github-pages`.
gem "github-pages", group: :jekyll_plugins

# If you have any plugins, put them here!
group :jekyll_plugins do
  gem "jekyll-feed", "~> 0.17.0"
  gem "jekyll-avatar", "~> 0.8.0"
  gem "jekyll-mentions", "~> 1.6.0"
  gem "jemoji", "~> 0.13.0"
  # gem "github-pages", "~> 232"
end

# Windows and JRuby does not include zoneinfo files, so bundle the tzinfo-data gem
# and associated library.
platforms :mingw, :x64_mingw, :mswin, :jruby do
  gem "tzinfo", ">= 1", "< 3"
  gem "tzinfo-data"
end

# Performance-booster for watching directories on Windows
# gem "wdm", "~> 0.1.0", :install_if => Gem.win_platform?

# kramdown v2 ships without the gfm parser by default. If you're using
# kramdown v1, comment out this line.
gem "kramdown-parser-gfm"

# Lock `http_parser.rb` gem to `v0.6.x` on JRuby builds since newer versions of the gem
# do not have a Java counterpart.
gem "http_parser.rb", "~> 0.6.0", :platforms => [:jruby]

# Development
group :development do
  gem "faraday-retry", "~> 2.0"
  # gem "webrick", "~> 1.8"
  # Troubleshooting Jekyll errors
  # See <https://primer.style/css/getting-started#troubleshooting-jekyll-errors>
  # gem 'jekyll-sass-converter', github: 'jekyll/jekyll-sass-converter'
  # gem 'sass-embedded'
end

# csv was loaded from the standard library, but will no longer be part of the default gems since Ruby 3.4.0.
# Add csv to your Gemfile or gemspec. Also contact author of jekyll-3.9.5 to add csv into its gemspec.
gem "csv", "~> 3.3"
